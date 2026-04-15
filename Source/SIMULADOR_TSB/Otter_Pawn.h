// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Otter_Pawn.generated.h"

class UOtter_ROS2UEConnector;

//Estados do sensor LiDAR
UENUM(BlueprintType)
enum class ELiDARHealth : uint8
{
	Healthy		UMETA(DisplayName = "Healthy"),
	Degraded	UMETA(DisplayName = "Degraded"),
	Frozen		UMETA(DisplayName = "Frozen"),
	Lost		UMETA(DisplayName = "Lost")
};

//Sensor LiDAR
USTRUCT(BlueprintType)
struct FLiDARBeam
{
	GENERATED_BODY()

	UPROPERTY()
	float AngleDeg = 0.0f;

	UPROPERTY()
	float Distance_m = 0.0f;

	UPROPERTY()
	bool bHit = false;

	UPROPERTY()
	bool bDropped = false;
};

USTRUCT(BlueprintType)
struct FLiDARSample
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FLiDARBeam> Beams;

	UPROPERTY()
	bool bValid = false;

	UPROPERTY()
	float TimeStamp_s = 0.0f;

	UPROPERTY()
	float ReleaseTime_s = 0.0f;

	UPROPERTY()
	ELiDARHealth Health = ELiDARHealth::Healthy;
};

//Estados do sensor AHRS
UENUM(BlueprintType)
enum class EAHRSHealth : uint8
{
	Healthy		UMETA(DisplayName = "Healthy"),
	Degraded	UMETA(DisplayName = "Degraded"),
	Frozen		UMETA(DisplayName = "Frozen"),
	Lost		UMETA(DisplayName = "Lost")
};

//Para o sensor AHRS
USTRUCT(BlueprintType)
struct FAHRSSample
{
	GENERATED_BODY()

	UPROPERTY()
	float Yaw_rad = 0.0f;

	UPROPERTY()
	float YawRate_radps = 0.0f;

	UPROPERTY()
	float SpecificForceX_mps2 = 0.0f;

	UPROPERTY()
	float SpecificForceY_mps2 = 0.0f;

	UPROPERTY()
	bool bValid = false;

	UPROPERTY()
	float TimeStamp_s = 0.0f;

	UPROPERTY()
	float ReleaseTime_s = 0.0f;

	UPROPERTY()
	EAHRSHealth Health = EAHRSHealth::Healthy;
};
//Estados do gps
UENUM(BlueprintType)
enum class EGPSHealth : uint8
{
	Healthy		UMETA(DisplayName = "Healthy"),
	Degraded	UMETA(DisplayName = "Degraded"),
	Frozen		UMETA(DisplayName = "Frozen"),
	Lost		UMETA(DisplayName = "Lost")
};

//Para o sensor GPS
USTRUCT(BlueprintType)
struct FGPSSample
{
	GENERATED_BODY()

	UPROPERTY()
	float X_m = 0.0f;

	UPROPERTY()
	float Y_m = 0.0f;

	UPROPERTY()
	bool bValid = false;

	UPROPERTY()
	float Error_m = 0.0f;

	UPROPERTY()
	float TimeStamp_s = 0.0f;

	UPROPERTY()
	float ReleaseTime_s = 0.0f;

	UPROPERTY()
	EGPSHealth Health = EGPSHealth::Healthy;
};
UCLASS()
class SIMULADOR_TSB_API AOtter_Pawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AOtter_Pawn();
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UStaticMeshComponent* HullMesh;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Components")
	UStaticMeshComponent* SensorMesh = nullptr;
	void OnMoveForward(float Value);
	void OnMoveRight(float Value);

	//ROS 2
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "ROS2")
	UOtter_ROS2UEConnector* RosBridge = nullptr;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


	// Sensor Sockets
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Sockets")
	FName GPSFrontSocketName = TEXT("GPSFront");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Sockets")
	FName GPSBackSocketName = TEXT("GPSBack");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Sockets")
	FName AHRSSocketName = TEXT("AHRS");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Sockets")
	FName LiDARSocketName = TEXT("Lidar");
	bool GetSensorSocketTransform(FName SocketName, FTransform& OutTransform) const;

	// LiDAR Sensor variaveis
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	bool bEnableLiDAR = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARUpdateRateHz = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARLatency_s = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARLatencyJitter_s = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARHorizontalFOV_deg = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	int32 LiDARNumBeams = 181;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARMinRange_m = 0.20f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARMaxRange_m = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARRangeSigma_m = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARDegradedRangeSigma_m = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARRangeResolution_m = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARBeamDropProbability = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARDegradedBeamDropProbability = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARDegradedProbability = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARFrozenProbability = 0.0002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARLostProbability = 0.0005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARFailureDurationMin_s = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARFailureDurationMax_s = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	bool bDrawLiDARDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARDebugDrawTime_s = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|LiDAR")
	float LiDARDebugLineThickness = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	bool bLiDARValid = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	ELiDARHealth LiDARHealth = ELiDARHealth::Healthy;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	float LiDARLastPublishedTime_s = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	int32 LiDARLastNumHits = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	int32 LiDARLastNumDropped = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	bool bLiDARSocketAvailable = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	TArray<FLiDARBeam> LiDARMeasuredBeams;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	TArray<FLiDARBeam> LiDARLastHealthyBeams;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	FVector LiDARSocketWorldLocation_cm = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	float LiDARSocketWorldYaw_deg = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	bool bLiDARFailing = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|LiDAR")
	float LiDARFailureTimeRemaining_s = 0.0f;

	float LiDARUpdateAccumulator = 0.0f;

	TArray<FLiDARSample> LiDARPendingSamples;

	// Funções para o LiDAR
	void UpdateLiDAR(float DeltaTime);
	void DrawLiDARDebug() const;

	FLiDARSample GenerateLiDARSample() const;
	void ReleasePendingLiDARSamples();

	ELiDARHealth PickNextLiDARFailureMode() const;
	float QuantizeRange(float Range_m) const;


	//AHRS variaveis
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSRealYawFromSocket_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	bool bAHRSSocketAvailable = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	bool bEnableAHRS = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSUpdateRateHz = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSLatency_s = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSLatencyJitter_s = 0.001f;

	// Ruído gaussiano nominal
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSYawRateSigma_radps = 0.0001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSSpecificForceSigma_mps2 = 0.01f;

	// Ruído gaussiano degradado
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSDegradedYawRateSigma_radps = 0.0005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSDegradedSpecificForceSigma_mps2 = 0.05f;

	// Bias random walk
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSGyroBiasWalkSigma_radps2 = 0.00002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSAccelBiasWalkSigma_mps3 = 0.001f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSGyroBias_radps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSAccelBiasX_mps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSAccelBiasY_mps2 = 0.0f;

	// Probabilidades de modos de falha
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSDegradedProbability = 0.0002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSFrozenProbability = 0.00005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSLostProbability = 0.00001f;

	// Duração dos modos de falha
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSFailureDurationMin_s = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSFailureDurationMax_s = 1.0f;

	//Correção
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|AHRS")
	float AHRSYawCorrectionGain = 0.8f;

	// Estado publicado
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float BodySpecificForceX_mps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float BodySpecificForceY_mps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float YawAccel_radps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSMeasuredYaw_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSMeasuredYawRate_radps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSMeasuredSpecificForceX_mps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSMeasuredSpecificForceY_mps2 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	bool bAHRSValid = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	EAHRSHealth AHRSHealth = EAHRSHealth::Healthy;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSLastPublishedTime_s = 0.0f;

	// Estado interno
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	bool bAHRSFailing = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSFailureTimeRemaining_s = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSEstimatedYaw_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|AHRS")
	float AHRSLastHealthyYaw_rad = 0.0f;

	float AHRSUpdateAccumulator = 0.0f;

	TArray<FAHRSSample> AHRSPendingSamples;

	// Funções AHRS
	void UpdateAHRS(float DeltaTime);
	void DrawAHRSDebug() const;

	FAHRSSample GenerateAHRSSample() const;
	void ReleasePendingAHRSSamples();

	EAHRSHealth PickNextAHRSFailureMode() const;
	
	// Simulação usando 3DOF (em principio vai usar o 6DOF 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
	float FixedTimeStep = 0.01f; // s

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
	int32 MaxSubstepsPerTick = 8;

	float TimeAccumulator = 0.0f;
	float InitialZ_cm = 0.0f;

	// Estado 3DOF
	// eta = [x, y, psi]
	// nu  = [u, v, r]
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaX_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaY_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaPsi_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuU_mps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuV_mps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuR_radps = 0.0f;


	// Parâmetros OTTER 3DOF
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|RigidBody")
	float Mass = 55.0f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|RigidBody")
	float InertiaYaw = 15.95f; // kg*m^2

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|AddedMass")
	float X_u_dot = 5.2815f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|AddedMass")
	float Y_v_dot = 82.5f; // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|AddedMass")
	float N_r_dot = 27.115f; // kg*m^2

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Damping")
	float X_u = 77.0f; // N/(m/s)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Damping")
	float Y_v = 137.0f; // N/(m/s)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Damping")
	float N_r = 46.0f; // N*m/(rad/s)

	// Thrusters

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrusters")
	float ThrusterHalfBeam = 0.3931f; // m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrusters")
	float MaxCommandCurrent = 20.0f; // A

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrusters")
	float MotorTimeConstant = 0.15f; // s

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thrusters")
	float CurrentLeft_A = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thrusters")
	float CurrentRight_A = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thrusters")
	float ThrustLeft_N = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thrusters")
	float ThrustRight_N = 0.0f;

	float InputForward = 0.0f;
	float InputRight = 0.0f;


	// GPS Sensor variaveis

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSRealX_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSRealY_m = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	bool bEnableGPS = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSUpdateRateHz = 10.0f;

	// Ruído gaussiano nominal
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSPosSigma_m = 0.5f;

	// Ruído gaussiano quando degradado
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSDegradedPosSigma_m = 1.5f;

	// Bias lento / random walk
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSBiasWalkSigma_mps = 0.005f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSBiasX_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSBiasY_m = 0.0f;

	// Latência
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSLatency_s = 0.10f;

	// Jitter opcional da latência
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSLatencyJitter_s = 0.01f;

	// Probabilidades de modos de falha
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSDegradedProbability = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSFrozenProbability = 0.0002f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSLostProbability = 0.00005f;

	// Duração dos modos de falha
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSFailureDurationMin_s = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|GPS")
	float GPSFailureDurationMax_s = 2.0f;

	// Saída publicada
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSMeasuredX_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSMeasuredY_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	bool bGPSValid = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSError_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	EGPSHealth GPSHealth = EGPSHealth::Healthy;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSLastPublishedTime_s = 0.0f;

	// Estado interno
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	bool bGPSFailing = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSFailureTimeRemaining_s = 0.0f;

	float GPSUpdateAccumulator = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Simulation")
	float SimulationTime_s = 0.0f;

	TArray<FGPSSample> GPSPendingSamples;

	// Última amostra saudável gerada, útil para modo Frozen
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSLastHealthyX_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors|GPS")
	float GPSLastHealthyY_m = 0.0f;

	// Funções GPS
	void UpdateGPS(float DeltaTime);
	void DrawGPSDebug() const;

	FGPSSample GenerateGPSSample() const;
	void ReleasePendingGPSSamples();

	float SampleGaussian(float Mean, float StdDev) const;
	EGPSHealth PickNextGPSFailureMode() const;

	// Funções internas

	// para os 3DOF
	void StepDynamics(float Dt);
	void UpdateActorTransformFromState();



	//6DOF
	void StepDynamics6DOF(float Dt);

	float CalculateT200Thrust(float CurrentAmps) const;
	float Interpolate1D(const float* X, const float* Y, int32 Count, float QueryX) const;

	
	// tentativa de upgrade
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
	bool bUse6DOF = false;

	// variaveis que falavam no 3DOF
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaZ_m = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaPhi_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float EtaTheta_rad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuW_mps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuP_radps = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
	float NuQ_radps = 0.0f;

	float InertiaRoll = 12.4643f;
	float InertiaPitch = 18.15f;

	float Z_w_dot = 55.0f;
	float K_p_dot = 2.4929f;
	float M_q_dot = 14.52f;

	float Z_w = 546.0f;
	float K_p = 54.0f;
	float M_q = 246.0f;

	//Forças restauradoras
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Restoring")
	float GravityZ = 9.81f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Restoring")
	float BuoyancyZ_N = 539.55f; // começa ≈ Mass * GravityZ

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Restoring")
	float GM_T = 0.4f; // altura metacêntrica transversal (m)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Restoring")
	float GM_L = 2.0f; // altura metacêntrica longitudinal (m)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrusters|Geometry")
	float ThrusterXOffset_m = -0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrusters|Geometry")
	float ThrusterZOffset_m = -0.15f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Lift")
	float HeaveLiftCoeff = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hydrodynamics|Lift")
	float PitchLiftCoeff = 1.0f;
};
