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
	// Construtor do componente.
	UOtter_ROS2UEConnector();

	// Inicializa socket UDP e, se ativo, arranca o ROS 2 ao iniciar o jogo.
	virtual void BeginPlay() override;

	// Fecha socket e termina o processo ROS 2 ao terminar o jogo.
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Envia uma mensagem GPS por UDP.
	void SendGPS(float X_m, float Y_m, bool bValid);

	// Envia uma mensagem AHRS por UDP.
	void SendAHRS(
		float Yaw_rad,
		float YawRate_radps,
		float SpecificForceX_mps2,
		float SpecificForceY_mps2,
		bool bValid
	);

	// Envia uma mensagem LiDAR por UDP.
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
	// Endereço IP de destino para envio UDP.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	FString TargetIp = TEXT("172.30.170.66");

	// Porto UDP de destino.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	int32 TargetPort = 5005;

	// Ativa ou desativa o envio de dados por UDP.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|UDP")
	bool bEnableSending = true;

	// Arranca automaticamente o ROS 2 no BeginPlay.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|AutoStart")
	bool bAutoStartROS2OnPlay = true;

	// Comando alternativo para arranque manual via WSL.
	// Atualmente pode ficar vazio se o arranque for construído diretamente no .cpp.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ROS2|AutoStart")
	FString WslLaunchCommand = TEXT("bash -ic '~/ros2_ws/start_ros2_demo.sh'");

private:
	// Socket UDP usado para enviar mensagens para o ROS 2.
	FSocket* UdpSocket = nullptr;

	// Handle do processo ROS 2 arrancado a partir do Unreal.
	FProcHandle ROS2ProcessHandle;

	// Cria o socket UDP, se ainda não existir.
	bool InitializeSocket();

	// Fecha e destrói o socket UDP.
	void CloseSocket();

	// Envia uma string JSON por UDP.
	bool SendString(const FString& Message);

	// Arranca o script ROS 2 através do WSL.
	void StartROS2OnPlay();

	// Termina o processo ROS 2 iniciado pelo Unreal.
	void StopROS2OnPlay();
};