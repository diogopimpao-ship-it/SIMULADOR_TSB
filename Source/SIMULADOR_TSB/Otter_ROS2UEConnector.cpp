#include "Otter_ROS2UEConnector.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"

UOtter_ROS2UEConnector::UOtter_ROS2UEConnector()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UOtter_ROS2UEConnector::BeginPlay()
{
	Super::BeginPlay();

	InitializeSocket();

	if (bAutoStartROS2OnPlay)
	{
		StartROS2OnPlay();
	}
}

void UOtter_ROS2UEConnector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopROS2OnPlay();
	CloseSocket();

	Super::EndPlay(EndPlayReason);
}

bool UOtter_ROS2UEConnector::InitializeSocket()
{
	if (UdpSocket)
	{
		return true;
	}

	UdpSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_DGram, TEXT("OtterROS2UDP"), false);

	if (!UdpSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: failed to create UDP socket"));
		return false;
	}

	UE_LOG(LogTemp, Warning, TEXT("ROS2UEConnector: UDP socket created. Target = %s:%d"), *TargetIp, TargetPort);
	return true;
}

void UOtter_ROS2UEConnector::CloseSocket()
{
	if (UdpSocket)
	{
		UdpSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(UdpSocket);
		UdpSocket = nullptr;
	}
}

bool UOtter_ROS2UEConnector::SendString(const FString& Message)
{
	if (!bEnableSending)
	{
		return false;
	}

	if (!UdpSocket && !InitializeSocket())
	{
		return false;
	}

	bool bIsValidIp = false;
	TSharedRef<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	Addr->SetIp(*TargetIp, bIsValidIp);
	Addr->SetPort(TargetPort);

	if (!bIsValidIp)
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: invalid target IP = %s"), *TargetIp);
		return false;
	}

	FTCHARToUTF8 Converter(*Message);
	int32 BytesSent = 0;

	const bool bSent = UdpSocket->SendTo(
		reinterpret_cast<const uint8*>(Converter.Get()),
		Converter.Length(),
		BytesSent,
		*Addr
	);

	if (!bSent)
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: failed to send UDP message"));
		return false;
	}

	return true;
}

void UOtter_ROS2UEConnector::SendGPS(float X_m, float Y_m, bool bValid)
{
	const FString JsonMessage = FString::Printf(
		TEXT("{\"type\":\"gps\",\"x\":%.6f,\"y\":%.6f,\"valid\":%s}"),
		X_m,
		Y_m,
		bValid ? TEXT("true") : TEXT("false")
	);

	SendString(JsonMessage);
}

void UOtter_ROS2UEConnector::SendAHRS(
	float Yaw_rad,
	float YawRate_radps,
	float SpecificForceX_mps2,
	float SpecificForceY_mps2,
	bool bValid
)
{
	const FString JsonMessage = FString::Printf(
		TEXT("{\"type\":\"ahrs\",\"yaw\":%.6f,\"yaw_rate\":%.6f,\"ax\":%.6f,\"ay\":%.6f,\"valid\":%s}"),
		Yaw_rad,
		YawRate_radps,
		SpecificForceX_mps2,
		SpecificForceY_mps2,
		bValid ? TEXT("true") : TEXT("false")
	);

	SendString(JsonMessage);
}

void UOtter_ROS2UEConnector::SendLiDAR(
	const TArray<float>& AnglesDeg,
	const TArray<float>& Distances_m,
	const TArray<bool>& Hits,
	const TArray<bool>& Dropped,
	float RangeMin_m,
	float RangeMax_m,
	bool bValid
)
{
	if (AnglesDeg.Num() != Distances_m.Num() ||
		AnglesDeg.Num() != Hits.Num() ||
		AnglesDeg.Num() != Dropped.Num())
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: SendLiDAR array size mismatch"));
		return;
	}

	FString BeamsJson = TEXT("[");

	for (int32 i = 0; i < AnglesDeg.Num(); ++i)
	{
		BeamsJson += FString::Printf(
			TEXT("{\"angle_deg\":%.6f,\"distance\":%.6f,\"hit\":%s,\"dropped\":%s}"),
			AnglesDeg[i],
			Distances_m[i],
			Hits[i] ? TEXT("true") : TEXT("false"),
			Dropped[i] ? TEXT("true") : TEXT("false")
		);

		if (i < AnglesDeg.Num() - 1)
		{
			BeamsJson += TEXT(",");
		}
	}

	BeamsJson += TEXT("]");

	const FString JsonMessage = FString::Printf(
		TEXT("{\"type\":\"lidar\",\"range_min\":%.6f,\"range_max\":%.6f,\"valid\":%s,\"beams\":%s}"),
		RangeMin_m,
		RangeMax_m,
		bValid ? TEXT("true") : TEXT("false"),
		*BeamsJson
	);

	SendString(JsonMessage);
}
void UOtter_ROS2UEConnector::StartROS2OnPlay()
{
	if (ROS2ProcessHandle.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("ROS2 already running."));
		return;
	}

	const FString ProjectDirWin = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
	const FString RootDirWin = FPaths::ConvertRelativePathToFull(FPaths::Combine(ProjectDirWin, TEXT("..")));

	FString RootDirWsl = RootDirWin;
	RootDirWsl.ReplaceInline(TEXT("\\"), TEXT("/"));

	if (RootDirWsl.Len() > 2 && RootDirWsl[1] == ':')
	{
		const TCHAR DriveLetter = FChar::ToLower(RootDirWsl[0]);
		RootDirWsl = FString::Printf(TEXT("/mnt/%c%s"), DriveLetter, *RootDirWsl.Mid(2));
	}

	const FString ScriptPathWsl = RootDirWsl / TEXT("ros2_ws/start_ros2_demo.sh");

	const FString Program = TEXT("wsl.exe");
	const FString Args = FString::Printf(
		TEXT("bash -lc \"chmod +x '%s' && '%s'\""),
		*ScriptPathWsl,
		*ScriptPathWsl
	);

	UE_LOG(LogTemp, Warning, TEXT("Launching ROS2 with: %s %s"), *Program, *Args);

	ROS2ProcessHandle = FPlatformProcess::CreateProc(
		*Program,
		*Args,
		true,
		false,
		false,
		nullptr,
		0,
		nullptr,
		nullptr
	);

	if (ROS2ProcessHandle.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("ROS2 launched on Play."));
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to launch ROS2 on Play."));
	}
}

void UOtter_ROS2UEConnector::StopROS2OnPlay()
{
	if (ROS2ProcessHandle.IsValid())
	{
		FPlatformProcess::TerminateProc(ROS2ProcessHandle, true);
		FPlatformProcess::CloseProc(ROS2ProcessHandle);
		ROS2ProcessHandle.Reset();

		UE_LOG(LogTemp, Warning, TEXT("ROS2 stopped on EndPlay."));
	}
}