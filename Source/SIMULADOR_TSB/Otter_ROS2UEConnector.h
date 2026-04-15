#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/PlatformProcess.h"
#include "Otter_ROS2UEConnector.generated.h"

class FSocket;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SIMULADOR_TSB_API UOtter_ROS2UEConnector : public UActorComponent
{
	GENERATED_BODY()

public:
	UOtter_ROS2UEConnector();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void SendGPS(float X_m, float Y_m, bool bValid);

	void SendAHRS(
		float Yaw_rad,
		float YawRate_radps,
		float SpecificForceX_mps2,
		float SpecificForceY_mps2,
		bool bValid
	);

	void SendLiDAR(
		const TArray<float>& AnglesDeg,
		const TArray<float>& Distances_m,
		const TArray<bool>& Hits,
		const TArray<bool>& Dropped,
		float RangeMin_m,
		float RangeMaxRange_m,
		bool bValid
	);

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	FString TargetIp = TEXT("172.30.170.66");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	int32 TargetPort = 5005;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	bool bEnableSending = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|AutoStart")
	bool bAutoStartROS2OnPlay = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|AutoStart")
	FString WslLaunchCommand = TEXT("bash -ic '~/ros2_ws/start_ros2_demo.sh'");

private:
	FSocket* UdpSocket = nullptr;
	FProcHandle ROS2ProcessHandle;

	bool InitializeSocket();
	void CloseSocket();
	bool SendString(const FString& Message);

	void StartROS2OnPlay();
	void StopROS2OnPlay();
};