#include "Otter_ROS2UEConnector.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"

UOtter_ROS2UEConnector::UOtter_ROS2UEConnector()
{
	// Este componente não precisa de Tick.
	PrimaryComponentTick.bCanEverTick = false;
}

void UOtter_ROS2UEConnector::BeginPlay()
{
	Super::BeginPlay();

	// Inicializa o socket UDP para envio de mensagens.
	InitializeSocket();

	// Arranca automaticamente o ROS 2, se essa opção estiver ativa.
	if (bAutoStartROS2OnPlay)
	{
		StartROS2OnPlay();
	}
}

void UOtter_ROS2UEConnector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Termina o processo ROS 2 e fecha o socket ao sair do jogo.
	StopROS2OnPlay();
	CloseSocket();

	Super::EndPlay(EndPlayReason);
}

bool UOtter_ROS2UEConnector::InitializeSocket()
{
	// Reutiliza o socket existente, se já tiver sido criado.
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
	// Fecha e liberta o socket UDP.
	if (UdpSocket)
	{
		UdpSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(UdpSocket);
		UdpSocket = nullptr;
	}
}

bool UOtter_ROS2UEConnector::SendString(const FString& Message)
{
	// Não envia dados se o envio estiver desativado.
	if (!bEnableSending)
	{
		return false;
	}

	// Garante que o socket existe antes de enviar.
	if (!UdpSocket && !InitializeSocket())
	{
		return false;
	}

	bool bIsValidIp = false;
	TSharedRef<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
	Addr->SetIp(*TargetIp, bIsValidIp);
	Addr->SetPort(TargetPort);

	// Valida o IP de destino.
	if (!bIsValidIp)
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: invalid target IP = %s"), *TargetIp);
		return false;
	}

	FTCHARToUTF8 Converter(*Message);
	int32 BytesSent = 0;

	// Envia a mensagem em formato UTF-8 por UDP.
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
	// Serializa os dados do GPS em JSON e envia por UDP.
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
	// Serializa os dados do AHRS em JSON e envia por UDP.
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
	// Garante que todos os vetores têm o mesmo número de beams.
	if (AnglesDeg.Num() != Distances_m.Num() ||
		AnglesDeg.Num() != Hits.Num() ||
		AnglesDeg.Num() != Dropped.Num())
	{
		UE_LOG(LogTemp, Error, TEXT("ROS2UEConnector: SendLiDAR array size mismatch"));
		return;
	}

	FString BeamsJson = TEXT("[");

	// Serializa cada beam do LiDAR para JSON.
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

	// Envia o scan completo do LiDAR em formato JSON.
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
	// Evita arrancar mais do que um processo ROS 2 em simultâneo.
	if (ROS2ProcessHandle.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("ROS2 already running."));
		return;
	}

	// Obtém a pasta do projeto no Windows.
	const FString ProjectDirWin = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());

	// Converte o caminho do projeto para formato compatível com WSL.
	FString ProjectDirWsl = ProjectDirWin;
	ProjectDirWsl.ReplaceInline(TEXT("\\"), TEXT("/"));

	if (ProjectDirWsl.Len() > 2 && ProjectDirWsl[1] == ':')
	{
		const TCHAR DriveLetter = FChar::ToLower(ProjectDirWsl[0]);
		ProjectDirWsl = FString::Printf(TEXT("/mnt/%c%s"), DriveLetter, *ProjectDirWsl.Mid(2));
	}

	// O script é procurado dentro da pasta do projeto: SIMULADOR_TSB/ros2_ws/
	const FString ScriptPathWsl = ProjectDirWsl / TEXT("ros2_ws/start_ros2_demo.sh");

	const FString Program = TEXT("wsl.exe");
	const FString Args = FString::Printf(
		TEXT("bash -lc \"chmod +x '%s' && '%s'\""),
		*ScriptPathWsl,
		*ScriptPathWsl
	);

	UE_LOG(LogTemp, Warning, TEXT("Launching ROS2 with: %s %s"), *Program, *Args);

	// Arranca o script ROS 2 no WSL.
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
	// Termina o processo ROS 2 iniciado no BeginPlay.
	if (ROS2ProcessHandle.IsValid())
	{
		FPlatformProcess::TerminateProc(ROS2ProcessHandle, true);
		FPlatformProcess::CloseProc(ROS2ProcessHandle);
		ROS2ProcessHandle.Reset();

		UE_LOG(LogTemp, Warning, TEXT("ROS2 stopped on EndPlay."));
	}
}