	// Fill out your copyright notice in the Description page of Project Settings.


	#include "Otter_Pawn.h"
	#include "Components/StaticMeshComponent.h"
	#include "Components/InputComponent.h"
	#include "Math/UnrealMathUtility.h"
	#include "DrawDebugHelpers.h"
	#include "Otter_ROS2UEConnector.h"

	class UStaticMeshComponent;
	class UInputComponent;


	// Sets default values
	AOtter_Pawn::AOtter_Pawn()
	{
 		// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
		PrimaryActorTick.bCanEverTick = true;

		HullMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HullMesh"));
		RootComponent = HullMesh;
		HullMesh->SetMobility(EComponentMobility::Movable);

		HullMesh->SetSimulatePhysics(false);
		HullMesh->SetEnableGravity(false);
		HullMesh->SetLinearDamping(0.0f);
		HullMesh->SetAngularDamping(0.0f);

		RosBridge = CreateDefaultSubobject<UOtter_ROS2UEConnector>(TEXT("RosBridge"));

	}

	// Called when the game starts or when spawned
	void AOtter_Pawn::BeginPlay()
	{
		Super::BeginPlay();
		// Garante que o Blueprint não reativa física
		if (HullMesh)
		{
			HullMesh->SetSimulatePhysics(false);
			HullMesh->SetEnableGravity(false);
		}
		SensorMesh = nullptr;

		TArray<UStaticMeshComponent*> MeshComponents;
		GetComponents<UStaticMeshComponent>(MeshComponents);

		for (UStaticMeshComponent* MeshComp : MeshComponents)
		{
			if (MeshComp && MeshComp->GetName() == TEXT("Cube"))
			{
				SensorMesh = MeshComp;
				break;
			}
		}

		if (!SensorMesh)
		{
			UE_LOG(LogTemp, Warning, TEXT("SensorMesh 'Cube' not found. Falling back to HullMesh."));
			SensorMesh = HullMesh;
		}

		if (SensorMesh && SensorMesh->GetStaticMesh())
		{
			UE_LOG(LogTemp, Warning, TEXT("SensorMesh in use = %s"), *SensorMesh->GetStaticMesh()->GetName());
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("SensorMesh is null or has no StaticMesh assigned."));
		}

		// Inicializa o estado a partir do transform atual do ator
		const FVector StartLocation_cm = GetActorLocation();
		const FRotator StartRotation_deg = GetActorRotation();

		EtaX_m = StartLocation_cm.X / 100.0f;
		EtaY_m = StartLocation_cm.Y / 100.0f;
		EtaPsi_rad = FMath::DegreesToRadians(StartRotation_deg.Yaw);

		InitialZ_cm = StartLocation_cm.Z;

		NuU_mps = 0.0f;
		NuV_mps = 0.0f;
		NuR_radps = 0.0f;

		//6dof
		bUse6DOF = true;
		EtaZ_m = 0.0f;
		EtaPhi_rad = 0.0f;
		EtaTheta_rad = 0.0f;
		NuW_mps = 0.0f;
		NuP_radps = 0.0f;
		NuQ_radps = 0.0f;

		TimeAccumulator = 0.0f;

		// começar o GPS
		GPSMeasuredX_m = EtaX_m;
		GPSMeasuredY_m = EtaY_m;
		GPSLastHealthyX_m = EtaX_m;
		GPSLastHealthyY_m = EtaY_m;

		bGPSValid = true;
		GPSError_m = 0.0f;
		GPSHealth = EGPSHealth::Healthy;
		GPSLastPublishedTime_s = 0.0f;

		GPSBiasX_m = 0.0f;
		GPSBiasY_m = 0.0f;

		bGPSFailing = false;
		GPSFailureTimeRemaining_s = 0.0f;
		GPSUpdateAccumulator = 0.0f;
		GPSPendingSamples.Empty();

		// Iniciar o AHRS
		AHRSMeasuredYaw_rad = EtaPsi_rad;
		AHRSMeasuredYawRate_radps = NuR_radps;
		AHRSMeasuredSpecificForceX_mps2 = 0.0f;
		AHRSMeasuredSpecificForceY_mps2 = 0.0f;

		bAHRSValid = true;
		AHRSHealth = EAHRSHealth::Healthy;
		AHRSLastPublishedTime_s = 0.0f;

		bAHRSFailing = false;
		AHRSFailureTimeRemaining_s = 0.0f;
		AHRSUpdateAccumulator = 0.0f;

		AHRSGyroBias_radps = 0.0f;
		AHRSAccelBiasX_mps2 = 0.0f;
		AHRSAccelBiasY_mps2 = 0.0f;

		AHRSEstimatedYaw_rad = EtaPsi_rad;
		AHRSLastHealthyYaw_rad = EtaPsi_rad;

		AHRSPendingSamples.Empty();

		//GPS, o erro estava muito alto
		GPSPosSigma_m = 0.10f;
		GPSDegradedPosSigma_m = 0.50f;
		GPSBiasWalkSigma_mps = 0.001f;

		GPSLatency_s = 0.0f;
		GPSLatencyJitter_s = 0.0f;

		GPSDegradedProbability = 0.0001f;
		GPSFrozenProbability = 0.0002f;
		GPSLostProbability = 0.00001f;

		GPSFailureDurationMin_s = 1.0f;
		GPSFailureDurationMax_s = 2.0f;

		// Iniciar o LiDAR
		bLiDARValid = true;
		LiDARHealth = ELiDARHealth::Healthy;
		LiDARLastPublishedTime_s = 0.0f;
		LiDARLastNumHits = 0;
		LiDARLastNumDropped = 0;

		bLiDARSocketAvailable = false;
		LiDARSocketWorldLocation_cm = FVector::ZeroVector;
		LiDARSocketWorldYaw_deg = 0.0f;

		bLiDARFailing = false;
		LiDARFailureTimeRemaining_s = 0.0f;
		LiDARUpdateAccumulator = 0.0f;

		LiDARMeasuredBeams.Empty();
		LiDARLastHealthyBeams.Empty();
		LiDARPendingSamples.Empty();
	}

	// Called every frame
	void AOtter_Pawn::Tick(float DeltaTime)
	{
		Super::Tick(DeltaTime);

		SimulationTime_s += DeltaTime;

		// Acumulador para passo fixo
		TimeAccumulator += DeltaTime;

		int32 NumSteps = 0;
		while (TimeAccumulator >= FixedTimeStep && NumSteps < MaxSubstepsPerTick)
		{
			if (bUse6DOF)
				StepDynamics6DOF(FixedTimeStep);
			else
				StepDynamics(FixedTimeStep);

			TimeAccumulator -= FixedTimeStep;
			++NumSteps;
		}

		UpdateActorTransformFromState();

		//GPS
		UpdateGPS(DeltaTime);
		DrawGPSDebug();

		//AHRS
		UpdateAHRS(DeltaTime);
		DrawAHRSDebug();

		// LiDAR
		UpdateLiDAR(DeltaTime);
		DrawLiDARDebug();


		//Verificar se está a usar os 6 ou os 3
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(
				200,
				0.0f,
				FColor::Blue,
				FString::Printf(
					TEXT("6DOF -> Z: %.3f m | Roll: %.2f deg | Pitch: %.2f deg | W: %.3f | P: %.3f | Q: %.3f"),
					EtaZ_m,
					FMath::RadiansToDegrees(EtaPhi_rad),
					FMath::RadiansToDegrees(EtaTheta_rad),
					NuW_mps,
					NuP_radps,
					NuQ_radps
				)
			);
		}
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(
				201,
				0.0f,
				FColor::White,
				FString::Printf(TEXT("bUse6DOF = %s"), bUse6DOF ? TEXT("TRUE") : TEXT("FALSE"))
			);
		}
	}

	// Called to bind functionality to input
	void AOtter_Pawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
	{
		Super::SetupPlayerInputComponent(PlayerInputComponent);

		if (!PlayerInputComponent)
		{
			return;
		}
		//Eixos do unreal
		PlayerInputComponent->BindAxis(TEXT("MoveForward"), this, &AOtter_Pawn::OnMoveForward);
		PlayerInputComponent->BindAxis(TEXT("MoveRight"), this, &AOtter_Pawn::OnMoveRight);

	}

	void AOtter_Pawn::OnMoveForward(float Value)
	{
		InputForward = FMath::Clamp(Value, -1.0f, 1.0f);
	}

	void AOtter_Pawn::OnMoveRight(float Value)
	{
		InputRight = FMath::Clamp(Value, -1.0f, 1.0f);
	}


	//Codigo para os 3DOF, deixei para o caso dos 6DOF não estarem a funcionar muito bem ele opta por usar os 3DOF
	void AOtter_Pawn::StepDynamics(float Dt)
	{
		//Massa efetiva para 3DOF
		const float m11 = Mass + X_u_dot;
		const float m22 = Mass + Y_v_dot;
		const float m33 = InertiaYaw + N_r_dot;

		if (m11 <= KINDA_SMALL_NUMBER || m22 <= KINDA_SMALL_NUMBER || m33 <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		//Misturador
		const float MixLeftCmd = FMath::Clamp(InputForward - InputRight, -1.0f, 1.0f);
		const float MixRightCmd = FMath::Clamp(InputForward + InputRight, -1.0f, 1.0f);

		const float TargetCurrentLeft_A = MixLeftCmd * MaxCommandCurrent;
		const float TargetCurrentRight_A = MixRightCmd * MaxCommandCurrent;

		// Dinâmica de 1ª ordem do atuador
		const float TauMotor = FMath::Max(MotorTimeConstant, 0.001f);
		const float Alpha = 1.0f - FMath::Exp(-Dt / TauMotor);

		CurrentLeft_A = FMath::Lerp(CurrentLeft_A, TargetCurrentLeft_A, Alpha);
		CurrentRight_A = FMath::Lerp(CurrentRight_A, TargetCurrentRight_A, Alpha);

		//Thrust dos T200 por LUT
		ThrustLeft_N = CalculateT200Thrust(CurrentLeft_A);
		ThrustRight_N = CalculateT200Thrust(CurrentRight_A);

		// Forças/momentos generalizados
		// tau = [X, Y, N]
		const float TauPropX = ThrustLeft_N + ThrustRight_N;
		const float TauPropY = 0.0f;
		const float TauPropN = ThrusterHalfBeam * (ThrustRight_N - ThrustLeft_N);

		// Damping linear
		const float TauDampX = -X_u * NuU_mps;
		const float TauDampY = -Y_v * NuV_mps;
		const float TauDampN = -N_r * NuR_radps;

		// Coriolis / centrípeta 3DOF
		const float TauCorX = m22 * NuV_mps * NuR_radps;
		const float TauCorY = -m11 * NuU_mps * NuR_radps;
		const float TauCorN = (m11 - m22) * NuU_mps * NuV_mps;

		// Soma total
		const float TauX = TauPropX + TauDampX + TauCorX;
		const float TauY = TauPropY + TauDampY + TauCorY;
		const float TauN = TauPropN + TauDampN + TauCorN;


		// 6) Dinâmica: nu_dot
		const float Udot = TauX / m11;
		const float Vdot = TauY / m22;
		const float Rdot = TauN / m33;

		// Semi-implicit Euler
		NuU_mps += Udot * Dt;
		NuV_mps += Vdot * Dt;
		NuR_radps += Rdot * Dt;

		// 7) Cinemática: eta_dot = J(psi) * nu
		const float Cpsi = FMath::Cos(EtaPsi_rad);
		const float Spsi = FMath::Sin(EtaPsi_rad);

		const float Xdot = NuU_mps * Cpsi - NuV_mps * Spsi;
		const float Ydot = NuU_mps * Spsi + NuV_mps * Cpsi;
		const float Psidot = NuR_radps;

		EtaX_m += Xdot * Dt;
		EtaY_m += Ydot * Dt;
		EtaPsi_rad += Psidot * Dt;
		EtaPsi_rad = FMath::UnwindRadians(EtaPsi_rad);

		BodySpecificForceX_mps2 = Udot - NuR_radps * NuV_mps;
		BodySpecificForceY_mps2 = Vdot + NuR_radps * NuU_mps;
		YawAccel_radps2 = Rdot;
	}

	void AOtter_Pawn::UpdateActorTransformFromState()
	{
		const float Z_cm = bUse6DOF
			? (InitialZ_cm + EtaZ_m * 100.0f)
			: InitialZ_cm;

		const FVector NewLocation_cm(
			EtaX_m * 100.0f,
			EtaY_m * 100.0f,
			Z_cm
		);

		// Unreal: FRotator(Pitch, Yaw, Roll) -> mapeia para (θ, ψ, φ)
		const FRotator NewRotation_deg(
			bUse6DOF ? FMath::RadiansToDegrees(EtaTheta_rad) : 0.0f,
			FMath::RadiansToDegrees(EtaPsi_rad),
			bUse6DOF ? FMath::RadiansToDegrees(EtaPhi_rad) : 0.0f
		);

		SetActorLocationAndRotation(NewLocation_cm, NewRotation_deg,
			false, nullptr, ETeleportType::TeleportPhysics);
	}

	float AOtter_Pawn::Interpolate1D(const float* X, const float* Y, int32 Count, float QueryX) const
	{
		if (X == nullptr || Y == nullptr || Count < 2)
		{
			return 0.0f;
		}

		if (QueryX <= X[0])
		{
			return Y[0];
		}

		if (QueryX >= X[Count - 1])
		{
			return Y[Count - 1];
		}

		for (int32 i = 0; i < Count - 1; ++i)
		{
			const float X0 = X[i];
			const float X1 = X[i + 1];

			if (QueryX >= X0 && QueryX <= X1)
			{
				const float Y0 = Y[i];
				const float Y1 = Y[i + 1];

				const float Den = X1 - X0;
				if (FMath::IsNearlyZero(Den))
				{
					return Y0;
				}

				const float T = (QueryX - X0) / Den;
				return FMath::Lerp(Y0, Y1, T);
			}
		}

		return Y[Count - 1];
	}


	//Função para os motores
	float AOtter_Pawn::CalculateT200Thrust(float CurrentAmps) const
	{
		static const float CurrentAxis_A[] =
		{
			0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f,
			10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f
		};

		static const float ForwardThrust_N[] =
		{
			0.00000f, 7.07267f, 10.76469f, 13.85620f, 16.39168f,
			18.94941f, 21.48949f, 23.84245f, 25.93311f, 27.77306f,
			29.91288f, 31.74916f, 33.67795f, 35.81705f, 37.46819f,
			39.05371f, 40.92846f, 42.26400f, 44.21529f, 45.34641f, 46.72615f
		};

		static const float ReverseThrust_N[] =
		{
			0.00000f, 5.69372f, 8.85195f, 10.98710f, 13.25569f,
			15.25739f, 17.30357f, 19.16885f, 20.53594f, 22.06316f,
			23.54550f, 25.15467f, 26.67368f, 27.95705f, 29.31466f,
			30.50766f, 31.94093f, 33.31715f, 34.39163f, 35.48451f, 36.49680f
		};

		constexpr int32 Count = UE_ARRAY_COUNT(CurrentAxis_A);

		static_assert(UE_ARRAY_COUNT(CurrentAxis_A) == UE_ARRAY_COUNT(ForwardThrust_N), "Forward LUT size mismatch");
		static_assert(UE_ARRAY_COUNT(CurrentAxis_A) == UE_ARRAY_COUNT(ReverseThrust_N), "Reverse LUT size mismatch");

		const float AbsCurrent = FMath::Clamp(FMath::Abs(CurrentAmps), 0.0f, MaxCommandCurrent);

		if (AbsCurrent < 0.05f)
		{
			return 0.0f;
		}

		if (CurrentAmps >= 0.0f)
		{
			return Interpolate1D(CurrentAxis_A, ForwardThrust_N, Count, AbsCurrent);
		}
		else
		{
			return -Interpolate1D(CurrentAxis_A, ReverseThrust_N, Count, AbsCurrent);
		}
	}

	//Ir buscar os sockets ao static mesh
	bool AOtter_Pawn::GetSensorSocketTransform(FName SocketName, FTransform& OutTransform) const
	{
		if (!SensorMesh)
		{
			UE_LOG(LogTemp, Error, TEXT("GetSensorSocketTransform: SensorMesh is null"));
			return false;
		}

		if (!SensorMesh->GetStaticMesh())
		{
			UE_LOG(LogTemp, Error, TEXT("GetSensorSocketTransform: SensorMesh has no StaticMesh assigned"));
			return false;
		}

		if (!SensorMesh->DoesSocketExist(SocketName))
		{
			UE_LOG(LogTemp, Error, TEXT("GetSensorSocketTransform: Socket '%s' does not exist on mesh '%s'"),
				*SocketName.ToString(),
				*SensorMesh->GetStaticMesh()->GetName());
			return false;
		}

		OutTransform = SensorMesh->GetSocketTransform(SocketName, RTS_World);
		return true;
	}
	void AOtter_Pawn::UpdateGPS(float DeltaTime)
	{
		if (!bEnableGPS)
		{
			return;
		}

		if (GPSUpdateRateHz <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		FTransform GPSTransform;
		if (GetSensorSocketTransform(GPSFrontSocketName, GPSTransform))
		{
			const FVector GPSWorldLocation_cm = GPSTransform.GetLocation();
			GPSRealX_m = GPSWorldLocation_cm.X / 100.0f;
			GPSRealY_m = GPSWorldLocation_cm.Y / 100.0f;
		}
		else
		{
			GPSRealX_m = EtaX_m;
			GPSRealY_m = EtaY_m;
		}


		// 1) Atualiza drift lento do bias
		GPSBiasX_m += SampleGaussian(0.0f, GPSBiasWalkSigma_mps) * DeltaTime;
		GPSBiasY_m += SampleGaussian(0.0f, GPSBiasWalkSigma_mps) * DeltaTime;

		// 2) Atualiza modo de falha persistente
		if (bGPSFailing)
		{
			GPSFailureTimeRemaining_s -= DeltaTime;

			if (GPSFailureTimeRemaining_s <= 0.0f)
			{
				bGPSFailing = false;
				GPSFailureTimeRemaining_s = 0.0f;
				GPSHealth = EGPSHealth::Healthy;
			}
		}

		// 3) Acumula tempo do sensor
		GPSUpdateAccumulator += DeltaTime;
		const float GPSPeriod = 1.0f / GPSUpdateRateHz;

		while (GPSUpdateAccumulator >= GPSPeriod)
		{
			GPSUpdateAccumulator -= GPSPeriod;

			// 4) Se está saudável, pode entrar num modo de falha
			if (!bGPSFailing)
			{
				const EGPSHealth CandidateMode = PickNextGPSFailureMode();

				if (CandidateMode != EGPSHealth::Healthy)
				{
					bGPSFailing = true;
					GPSHealth = CandidateMode;
					GPSFailureTimeRemaining_s = FMath::FRandRange(
						GPSFailureDurationMin_s,
						GPSFailureDurationMax_s
					);
				}
				else
				{
					GPSHealth = EGPSHealth::Healthy;
				}
			}

			// 5) Gera amostra e mete na fila
			FGPSSample NewSample = GenerateGPSSample();

			// Guarda última saudável, para modo Frozen futuro
			if (NewSample.bValid && GPSHealth != EGPSHealth::Frozen)
			{
				GPSLastHealthyX_m = NewSample.X_m;
				GPSLastHealthyY_m = NewSample.Y_m;
			}

			GPSPendingSamples.Add(NewSample);
		}

		// 6) Liberta amostras prontas
		ReleasePendingGPSSamples();
	
	}
	float AOtter_Pawn::SampleGaussian(float Mean, float StdDev) const
	{
		if (StdDev <= KINDA_SMALL_NUMBER)
		{
			return Mean;
		}

		const float U1 = FMath::Clamp(FMath::FRand(), KINDA_SMALL_NUMBER, 1.0f);
		const float U2 = FMath::FRand();

		const float Mag = StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1));
		const float Z0 = Mag * FMath::Cos(2.0f * PI * U2);

		return Mean + Z0;
	}
	EGPSHealth AOtter_Pawn::PickNextGPSFailureMode() const
	{
		const float Total =
			GPSDegradedProbability +
			GPSFrozenProbability +
			GPSLostProbability;

		if (Total <= KINDA_SMALL_NUMBER)
		{
			return EGPSHealth::Healthy;
		}

		const float R = FMath::FRandRange(0.0f, Total);

		if (R < GPSDegradedProbability)
		{
			return EGPSHealth::Degraded;
		}

		if (R < GPSDegradedProbability + GPSFrozenProbability)
		{
			return EGPSHealth::Frozen;
		}

		return EGPSHealth::Lost;
	}

	FGPSSample AOtter_Pawn::GenerateGPSSample() const
	{
		FGPSSample Sample;

		Sample.TimeStamp_s = SimulationTime_s;

		const float LatencyNoise = SampleGaussian(0.0f, GPSLatencyJitter_s);
		Sample.ReleaseTime_s = SimulationTime_s + FMath::Max(0.0f, GPSLatency_s + LatencyNoise);

		Sample.Health = GPSHealth;

		FTransform GPSTransform;
		if (!GetSensorSocketTransform(GPSFrontSocketName, GPSTransform))
		{
			Sample.bValid = false;
			Sample.Health = EGPSHealth::Lost;
			Sample.X_m = GPSMeasuredX_m;
			Sample.Y_m = GPSMeasuredY_m;
			Sample.Error_m = 0.0f;
			return Sample;
		}

		const FVector GPSWorldLocation_cm = GPSTransform.GetLocation();
		const float RealX_m = GPSWorldLocation_cm.X / 100.0f;
		const float RealY_m = GPSWorldLocation_cm.Y / 100.0f;

		// LOST = sem medição válida
		if (GPSHealth == EGPSHealth::Lost)
		{
			Sample.bValid = false;
			Sample.X_m = GPSMeasuredX_m;
			Sample.Y_m = GPSMeasuredY_m;
			Sample.Error_m = 0.0f;
			return Sample;
		}

		// FROZEN = última posição saudável, continua válida
		if (GPSHealth == EGPSHealth::Frozen)
		{
			Sample.bValid = true;
			Sample.X_m = GPSLastHealthyX_m;
			Sample.Y_m = GPSLastHealthyY_m;

			const float ErrorX = Sample.X_m - GPSRealX_m;
			const float ErrorY = Sample.Y_m - GPSRealY_m;
			Sample.Error_m = FMath::Sqrt(ErrorX * ErrorX + ErrorY * ErrorY);

			return Sample;
		}

		// HEALTHY / DEGRADED
		const float Sigma =
			(GPSHealth == EGPSHealth::Degraded)
			? GPSDegradedPosSigma_m
			: GPSPosSigma_m;

		const float NoiseX = SampleGaussian(0.0f, Sigma);
		const float NoiseY = SampleGaussian(0.0f, Sigma);

		Sample.X_m = GPSRealX_m + GPSBiasX_m + NoiseX;
		Sample.Y_m = GPSRealY_m + GPSBiasY_m + NoiseY;
		Sample.bValid = true;

		const float ErrorX = Sample.X_m - GPSRealX_m;
		const float ErrorY = Sample.Y_m - GPSRealY_m;
		Sample.Error_m = FMath::Sqrt(ErrorX * ErrorX + ErrorY * ErrorY);

		return Sample;
	}

	EAHRSHealth AOtter_Pawn::PickNextAHRSFailureMode() const
	{
		const float Total =
			AHRSDegradedProbability +
			AHRSFrozenProbability +
			AHRSLostProbability;

		if (Total <= KINDA_SMALL_NUMBER)
		{
			return EAHRSHealth::Healthy;
		}

		const float R = FMath::FRandRange(0.0f, Total);

		if (R < AHRSDegradedProbability)
		{
			return EAHRSHealth::Degraded;
		}

		if (R < AHRSDegradedProbability + AHRSFrozenProbability)
		{
			return EAHRSHealth::Frozen;
		}

		return EAHRSHealth::Lost;
	}

	void AOtter_Pawn::ReleasePendingGPSSamples()
	{
		while (GPSPendingSamples.Num() > 0)
		{
			const FGPSSample& Sample = GPSPendingSamples[0];

			if (Sample.ReleaseTime_s > SimulationTime_s)
			{
				break;
			}

			GPSMeasuredX_m = Sample.X_m;
			GPSMeasuredY_m = Sample.Y_m;
			bGPSValid = Sample.bValid;
			GPSError_m = Sample.Error_m;
			GPSHealth = Sample.Health;
			GPSLastPublishedTime_s = Sample.TimeStamp_s;

			//ROS 2 publicar
			if (RosBridge)
			{
				RosBridge->SendGPS(GPSMeasuredX_m, GPSMeasuredY_m, bGPSValid);
			}

			GPSPendingSamples.RemoveAt(0);
		}
	}
	FAHRSSample AOtter_Pawn::GenerateAHRSSample() const
	{
		FAHRSSample Sample;

		Sample.TimeStamp_s = SimulationTime_s;
		Sample.ReleaseTime_s = SimulationTime_s + FMath::Max(0.0f, AHRSLatency_s + SampleGaussian(0.0f, AHRSLatencyJitter_s));
		Sample.Health = AHRSHealth;
		FTransform AHRSTransform;
		if (!GetSensorSocketTransform(AHRSSocketName, AHRSTransform))
		{
			Sample.bValid = false;
			Sample.Health = EAHRSHealth::Lost;
			Sample.Yaw_rad = AHRSMeasuredYaw_rad;
			Sample.YawRate_radps = AHRSMeasuredYawRate_radps;
			Sample.SpecificForceX_mps2 = AHRSMeasuredSpecificForceX_mps2;
			Sample.SpecificForceY_mps2 = AHRSMeasuredSpecificForceY_mps2;
			return Sample;
		}

		// LOST = sem medição válida
		if (AHRSHealth == EAHRSHealth::Lost)
		{
			Sample.bValid = false;
			Sample.Yaw_rad = AHRSMeasuredYaw_rad;
			Sample.YawRate_radps = AHRSMeasuredYawRate_radps;
			Sample.SpecificForceX_mps2 = AHRSMeasuredSpecificForceX_mps2;
			Sample.SpecificForceY_mps2 = AHRSMeasuredSpecificForceY_mps2;
			return Sample;
		}

		// FROZEN = mantém última estimativa saudável
		if (AHRSHealth == EAHRSHealth::Frozen)
		{
			Sample.bValid = true;
			Sample.Yaw_rad = AHRSLastHealthyYaw_rad;
			Sample.YawRate_radps = 0.0f;
			Sample.SpecificForceX_mps2 = AHRSMeasuredSpecificForceX_mps2;
			Sample.SpecificForceY_mps2 = AHRSMeasuredSpecificForceY_mps2;
			return Sample;
		}

		const float GyroSigma =
			(AHRSHealth == EAHRSHealth::Degraded)
			? AHRSDegradedYawRateSigma_radps
			: AHRSYawRateSigma_radps;

		const float AccelSigma =
			(AHRSHealth == EAHRSHealth::Degraded)
			? AHRSDegradedSpecificForceSigma_mps2
			: AHRSSpecificForceSigma_mps2;

		const float MeasuredYawRate =
			NuR_radps +
			AHRSGyroBias_radps +
			SampleGaussian(0.0f, GyroSigma);

		Sample.YawRate_radps = MeasuredYawRate;

		//o yaw publicado vai ser a estimativa integrada
		Sample.Yaw_rad = AHRSEstimatedYaw_rad;

		Sample.SpecificForceX_mps2 =
			BodySpecificForceX_mps2 +
			AHRSAccelBiasX_mps2 +
			SampleGaussian(0.0f, AccelSigma);

		Sample.SpecificForceY_mps2 =
			BodySpecificForceY_mps2 +
			AHRSAccelBiasY_mps2 +
			SampleGaussian(0.0f, AccelSigma);

		Sample.bValid = true;
		return Sample;
	}
	void AOtter_Pawn::ReleasePendingAHRSSamples()
	{
		while (AHRSPendingSamples.Num() > 0)
		{
			const FAHRSSample& Sample = AHRSPendingSamples[0];

			if (Sample.ReleaseTime_s > SimulationTime_s)
			{
				break;
			}

			AHRSMeasuredYaw_rad = Sample.Yaw_rad;
			AHRSMeasuredYawRate_radps = Sample.YawRate_radps;
			AHRSMeasuredSpecificForceX_mps2 = Sample.SpecificForceX_mps2;
			AHRSMeasuredSpecificForceY_mps2 = Sample.SpecificForceY_mps2;
			bAHRSValid = Sample.bValid;
			AHRSHealth = Sample.Health;
			AHRSLastPublishedTime_s = Sample.TimeStamp_s;

			if (RosBridge)
			{
				RosBridge->SendAHRS(
					AHRSMeasuredYaw_rad,
					AHRSMeasuredYawRate_radps,
					AHRSMeasuredSpecificForceX_mps2,
					AHRSMeasuredSpecificForceY_mps2,
					bAHRSValid
				);
			}

			AHRSPendingSamples.RemoveAt(0);
		}
	}
	void AOtter_Pawn::UpdateAHRS(float DeltaTime)
	{
		if (!bEnableAHRS)
		{
			return;
		}

		if (AHRSUpdateRateHz <= KINDA_SMALL_NUMBER)
		{
			return;
		}
		FTransform AHRSTransform;
		if (GetSensorSocketTransform(AHRSSocketName, AHRSTransform))
		{
			bAHRSSocketAvailable = true;
			AHRSRealYawFromSocket_rad = FMath::DegreesToRadians(AHRSTransform.Rotator().Yaw);
		}
		else
		{
			bAHRSSocketAvailable = false;
			AHRSRealYawFromSocket_rad = EtaPsi_rad;
		}
		// 1) Atualiza bias lento
		AHRSGyroBias_radps += SampleGaussian(0.0f, AHRSGyroBiasWalkSigma_radps2) * DeltaTime;
		AHRSAccelBiasX_mps2 += SampleGaussian(0.0f, AHRSAccelBiasWalkSigma_mps3) * DeltaTime;
		AHRSAccelBiasY_mps2 += SampleGaussian(0.0f, AHRSAccelBiasWalkSigma_mps3) * DeltaTime;

		// 2) Atualiza falha persistente
		if (bAHRSFailing)
		{
			AHRSFailureTimeRemaining_s -= DeltaTime;

			if (AHRSFailureTimeRemaining_s <= 0.0f)
			{
				bAHRSFailing = false;
				AHRSFailureTimeRemaining_s = 0.0f;
				AHRSHealth = EAHRSHealth::Healthy;
			}
		}

		// 3) Acumula tempo
		AHRSUpdateAccumulator += DeltaTime;
		const float AHRSPeriod = 1.0f / AHRSUpdateRateHz;

		while (AHRSUpdateAccumulator >= AHRSPeriod)
		{
			AHRSUpdateAccumulator -= AHRSPeriod;

			// 4) Se saudável, pode entrar em falha
			if (!bAHRSFailing)
			{
				const EAHRSHealth CandidateMode = PickNextAHRSFailureMode();

				if (CandidateMode != EAHRSHealth::Healthy)
				{
					bAHRSFailing = true;
					AHRSHealth = CandidateMode;
					AHRSFailureTimeRemaining_s = FMath::FRandRange(
						AHRSFailureDurationMin_s,
						AHRSFailureDurationMax_s
					);
				}
				else
				{
					AHRSHealth = EAHRSHealth::Healthy;
				}
			}

			// 5) Integra yaw estimado usando gyro medido,
			// exceto no modo Lost
			if (AHRSHealth != EAHRSHealth::Lost)
			{
				const float GyroSigma =
					(AHRSHealth == EAHRSHealth::Degraded)
					? AHRSDegradedYawRateSigma_radps
					: AHRSYawRateSigma_radps;

				float MeasuredYawRate =
					NuR_radps +
					AHRSGyroBias_radps +
					SampleGaussian(0.0f, GyroSigma);

				if (AHRSHealth == EAHRSHealth::Frozen)
				{
					MeasuredYawRate = 0.0f;
				}

				// Propagação por gyro
				AHRSEstimatedYaw_rad += MeasuredYawRate * AHRSPeriod;

				// Correção lenta para impedir deriva excessiva
				if (AHRSHealth != EAHRSHealth::Frozen)
				{
					const float YawError = FMath::FindDeltaAngleRadians(AHRSEstimatedYaw_rad, EtaPsi_rad);
					AHRSEstimatedYaw_rad += AHRSYawCorrectionGain * YawError * AHRSPeriod;
				}

				AHRSEstimatedYaw_rad = FMath::UnwindRadians(AHRSEstimatedYaw_rad);

				if (AHRSHealth != EAHRSHealth::Frozen)
				{
					AHRSLastHealthyYaw_rad = AHRSEstimatedYaw_rad;
				}
			}

			AHRSPendingSamples.Add(GenerateAHRSSample());
		}

		ReleasePendingAHRSSamples();
	}

	ELiDARHealth AOtter_Pawn::PickNextLiDARFailureMode() const
	{
		const float Total =
			LiDARDegradedProbability +
			LiDARFrozenProbability +
			LiDARLostProbability;

		if (Total <= KINDA_SMALL_NUMBER)
		{
			return ELiDARHealth::Healthy;
		}

		const float R = FMath::FRandRange(0.0f, Total);

		if (R < LiDARDegradedProbability)
		{
			return ELiDARHealth::Degraded;
		}

		if (R < LiDARDegradedProbability + LiDARFrozenProbability)
		{
			return ELiDARHealth::Frozen;
		}

		return ELiDARHealth::Lost;
	}

	float AOtter_Pawn::QuantizeRange(float Range_m) const
	{
		if (LiDARRangeResolution_m <= KINDA_SMALL_NUMBER)
		{
			return Range_m;
		}

		const float Steps = FMath::RoundToFloat(Range_m / LiDARRangeResolution_m);
		return Steps * LiDARRangeResolution_m;
	}

	FLiDARSample AOtter_Pawn::GenerateLiDARSample() const
	{
		FLiDARSample Sample;
		Sample.TimeStamp_s = SimulationTime_s;
		Sample.ReleaseTime_s = SimulationTime_s + FMath::Max(
			0.0f,
			LiDARLatency_s + SampleGaussian(0.0f, LiDARLatencyJitter_s)
		);
		Sample.Health = LiDARHealth;

		FTransform LiDARTransform;
		if (!GetSensorSocketTransform(LiDARSocketName, LiDARTransform))
		{
			Sample.bValid = false;
			Sample.Health = ELiDARHealth::Lost;
			return Sample;
		}

		// LOST = sem scan válido
		if (LiDARHealth == ELiDARHealth::Lost)
		{
			Sample.bValid = false;
			return Sample;
		}

		// FROZEN = repete último scan saudável
		if (LiDARHealth == ELiDARHealth::Frozen)
		{
			Sample.bValid = true;
			Sample.Beams = LiDARLastHealthyBeams;
			return Sample;
		}

		if (!GetWorld() || LiDARNumBeams <= 0)
		{
			Sample.bValid = false;
			return Sample;
		}

		const float HalfFOV = 0.5f * LiDARHorizontalFOV_deg;
		const float RangeSigma =
			(LiDARHealth == ELiDARHealth::Degraded)
			? LiDARDegradedRangeSigma_m
			: LiDARRangeSigma_m;

		const float DropProb =
			(LiDARHealth == ELiDARHealth::Degraded)
			? LiDARDegradedBeamDropProbability
			: LiDARBeamDropProbability;

		const FVector SensorLocation = LiDARTransform.GetLocation();
		const float SensorYawDeg = LiDARTransform.Rotator().Yaw;

		FCollisionQueryParams QueryParams;
		QueryParams.AddIgnoredActor(this);

		for (int32 i = 0; i < LiDARNumBeams; ++i)
		{
			FLiDARBeam Beam;

			const float Alpha =
				(LiDARNumBeams == 1)
				? 0.5f
				: static_cast<float>(i) / static_cast<float>(LiDARNumBeams - 1);

			Beam.AngleDeg = FMath::Lerp(-HalfFOV, HalfFOV, Alpha);

			const float BeamYawDeg = SensorYawDeg + Beam.AngleDeg;
			const FVector Direction = FRotator(0.0f, BeamYawDeg, 0.0f).Vector();

			const FVector TraceStart = SensorLocation;
			const FVector TraceEnd = TraceStart + Direction * (LiDARMaxRange_m * 100.0f);

			if (FMath::FRand() < DropProb)
			{
				Beam.bDropped = true;
				Beam.bHit = false;
				Beam.Distance_m = LiDARMaxRange_m;
				Sample.Beams.Add(Beam);
				continue;
			}

			FHitResult Hit;
			const bool bHit = GetWorld()->LineTraceSingleByChannel(
				Hit,
				TraceStart,
				TraceEnd,
				ECC_Visibility,
				QueryParams
			);

			Beam.bDropped = false;
			Beam.bHit = bHit;

			if (bHit)
			{
				float Distance_m = Hit.Distance / 100.0f;
				Distance_m += SampleGaussian(0.0f, RangeSigma);
				Distance_m = FMath::Clamp(Distance_m, LiDARMinRange_m, LiDARMaxRange_m);
				Distance_m = QuantizeRange(Distance_m);
				Beam.Distance_m = Distance_m;
			}
			else
			{
				Beam.Distance_m = LiDARMaxRange_m;
			}

			Sample.Beams.Add(Beam);
		}

		Sample.bValid = true;
		return Sample;
	}

		void AOtter_Pawn::ReleasePendingLiDARSamples()
		{
			while (LiDARPendingSamples.Num() > 0)
			{
				const FLiDARSample& Sample = LiDARPendingSamples[0];

				if (Sample.ReleaseTime_s > SimulationTime_s)
				{
					break;
				}

				LiDARMeasuredBeams = Sample.Beams;
				bLiDARValid = Sample.bValid;
				LiDARHealth = Sample.Health;
				LiDARLastPublishedTime_s = Sample.TimeStamp_s;

				int32 NumHits = 0;
				int32 NumDropped = 0;

				for (const FLiDARBeam& Beam : LiDARMeasuredBeams)
				{
					if (Beam.bHit)
					{
						++NumHits;
					}
					if (Beam.bDropped)
					{
						++NumDropped;
					}
				}

				LiDARLastNumHits = NumHits;
				LiDARLastNumDropped = NumDropped;

				if (Sample.bValid && LiDARHealth != ELiDARHealth::Frozen)
				{
					LiDARLastHealthyBeams = Sample.Beams;
				}

				if (RosBridge)
				{
					TArray<float> AnglesDeg;
					TArray<float> Distances_m;
					TArray<bool> Hits;
					TArray<bool> Dropped;

					AnglesDeg.Reserve(LiDARMeasuredBeams.Num());
					Distances_m.Reserve(LiDARMeasuredBeams.Num());
					Hits.Reserve(LiDARMeasuredBeams.Num());
					Dropped.Reserve(LiDARMeasuredBeams.Num());

					for (const FLiDARBeam& Beam : LiDARMeasuredBeams)
					{
						AnglesDeg.Add(Beam.AngleDeg);
						Distances_m.Add(Beam.Distance_m);
						Hits.Add(Beam.bHit);
						Dropped.Add(Beam.bDropped);
					}

					RosBridge->SendLiDAR(
						AnglesDeg,
						Distances_m,
						Hits,
						Dropped,
						LiDARMinRange_m,
						LiDARMaxRange_m,
						bLiDARValid
					);
				}

				LiDARPendingSamples.RemoveAt(0);
			}
		}

	void AOtter_Pawn::UpdateLiDAR(float DeltaTime)
	{
		if (!bEnableLiDAR)
		{
			return;
		}

		if (LiDARUpdateRateHz <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		FTransform LiDARTransform;
		if (GetSensorSocketTransform(LiDARSocketName, LiDARTransform))
		{
			bLiDARSocketAvailable = true;
			LiDARSocketWorldLocation_cm = LiDARTransform.GetLocation();
			LiDARSocketWorldYaw_deg = LiDARTransform.Rotator().Yaw;
		}
		else
		{
			bLiDARSocketAvailable = false;
			LiDARSocketWorldLocation_cm = GetActorLocation();
			LiDARSocketWorldYaw_deg = GetActorRotation().Yaw;
		}

		if (bLiDARFailing)
		{
			LiDARFailureTimeRemaining_s -= DeltaTime;

			if (LiDARFailureTimeRemaining_s <= 0.0f)
			{
				bLiDARFailing = false;
				LiDARFailureTimeRemaining_s = 0.0f;
				LiDARHealth = ELiDARHealth::Healthy;
			}
		}

		LiDARUpdateAccumulator += DeltaTime;
		const float LiDARPeriod = 1.0f / LiDARUpdateRateHz;

		while (LiDARUpdateAccumulator >= LiDARPeriod)
		{
			LiDARUpdateAccumulator -= LiDARPeriod;

			if (!bLiDARFailing)
			{
				const ELiDARHealth CandidateMode = PickNextLiDARFailureMode();

				if (CandidateMode != ELiDARHealth::Healthy)
				{
					bLiDARFailing = true;
					LiDARHealth = CandidateMode;
					LiDARFailureTimeRemaining_s = FMath::FRandRange(
						LiDARFailureDurationMin_s,
						LiDARFailureDurationMax_s
					);
				}
				else
				{
					LiDARHealth = ELiDARHealth::Healthy;
				}
			}

			LiDARPendingSamples.Add(GenerateLiDARSample());
		}

		ReleasePendingLiDARSamples();
	}

	void AOtter_Pawn::DrawLiDARDebug() const
	{
		if (!GEngine)
		{
			return;
		}

		const TCHAR* HealthText = TEXT("Healthy");
		switch (LiDARHealth)
		{
		case ELiDARHealth::Healthy:  HealthText = TEXT("Healthy"); break;
		case ELiDARHealth::Degraded: HealthText = TEXT("Degraded"); break;
		case ELiDARHealth::Frozen:   HealthText = TEXT("Frozen"); break;
		case ELiDARHealth::Lost:     HealthText = TEXT("Lost"); break;
		default: break;
		}

		GEngine->AddOnScreenDebugMessage(
			120,
			0.0f,
			bLiDARValid ? FColor::Cyan : FColor::Red,
			FString::Printf(
				TEXT("LiDAR -> Valid: %s | Health: %s | Hits: %d | Dropped: %d | Beams: %d"),
				bLiDARValid ? TEXT("YES") : TEXT("NO"),
				HealthText,
				LiDARLastNumHits,
				LiDARLastNumDropped,
				LiDARMeasuredBeams.Num()
			)
		);

		GEngine->AddOnScreenDebugMessage(
			121,
			0.0f,
			FColor::White,
			FString::Printf(
				TEXT("LiDAR -> Socket: %s | Rate: %.1f Hz | Lat: %.3f s | FOV: %.1f deg | MaxRange: %.1f m"),
				bLiDARSocketAvailable ? TEXT("OK") : TEXT("MISSING"),
				LiDARUpdateRateHz,
				LiDARLatency_s,
				LiDARHorizontalFOV_deg,
				LiDARMaxRange_m
			)
		);

		if (!bDrawLiDARDebug || !bLiDARValid || !GetWorld())
		{
			return;
		}

		const FVector SensorLocation = LiDARSocketWorldLocation_cm;
		const float SensorYawDeg = LiDARSocketWorldYaw_deg;

		for (const FLiDARBeam& Beam : LiDARMeasuredBeams)
		{
			const float BeamYawDeg = SensorYawDeg + Beam.AngleDeg;
			const FVector Direction = FRotator(0.0f, BeamYawDeg, 0.0f).Vector();
			const FVector End = SensorLocation + Direction * (Beam.Distance_m * 100.0f);

			FColor LineColor = FColor::Yellow;
			if (Beam.bDropped)
			{
				LineColor = FColor::Red;
			}
			else if (Beam.bHit)
			{
				LineColor = FColor::Green;
			}

			DrawDebugLine(
				GetWorld(),
				SensorLocation,
				End,
				LineColor,
				false,
				LiDARDebugDrawTime_s,
				0,
				LiDARDebugLineThickness
			);
		}
	}

	void AOtter_Pawn::DrawAHRSDebug() const
	{
		if (!GEngine)
		{
			return;
		}

		const TCHAR* HealthText = TEXT("Healthy");
		switch (AHRSHealth)
		{
		case EAHRSHealth::Healthy:  HealthText = TEXT("Healthy"); break;
		case EAHRSHealth::Degraded: HealthText = TEXT("Degraded"); break;
		case EAHRSHealth::Frozen:   HealthText = TEXT("Frozen"); break;
		case EAHRSHealth::Lost:     HealthText = TEXT("Lost"); break;
		default: break;
		}

		if (bAHRSValid)
		{
			GEngine->AddOnScreenDebugMessage(
				110,
				0.0f,
				(AHRSHealth == EAHRSHealth::Degraded) ? FColor::Orange : FColor::Emerald,
				FString::Printf(
					TEXT("AHRS -> Yaw: %.2f deg | YawRate: %.3f rad/s | Fx: %.2f | Fy: %.2f | VALID"),
					FMath::RadiansToDegrees(AHRSMeasuredYaw_rad),
					AHRSMeasuredYawRate_radps,
					AHRSMeasuredSpecificForceX_mps2,
					AHRSMeasuredSpecificForceY_mps2
				)
			);
		}
		else
		{
			GEngine->AddOnScreenDebugMessage(
				110,
				0.0f,
				FColor::Red,
				FString::Printf(TEXT("AHRS -> INVALID / SENSOR LOST"))
			);
		}

		GEngine->AddOnScreenDebugMessage(
			111,
			0.0f,
			FColor::Silver,
			FString::Printf(
				TEXT("AHRS -> Rate: %.1f Hz | Lat: %.3f s | Pending: %d | Health: %s"),
				AHRSUpdateRateHz,
				AHRSLatency_s,
				AHRSPendingSamples.Num(),
				HealthText
			)
		);

		GEngine->AddOnScreenDebugMessage(
			112,
			0.0f,
			FColor::Cyan,
			FString::Printf(
				TEXT("REAL AHRS SOCKET -> Yaw: %.2f deg | YawRate: %.3f rad/s | Fx: %.2f | Fy: %.2f | Socket: %s"),
				FMath::RadiansToDegrees(AHRSRealYawFromSocket_rad),
				NuR_radps,
				BodySpecificForceX_mps2,
				BodySpecificForceY_mps2,
				bAHRSSocketAvailable ? TEXT("OK") : TEXT("MISSING")
			)
		);

		GEngine->AddOnScreenDebugMessage(
			113,
			0.0f,
			FColor::White,
			FString::Printf(
				TEXT("AHRS -> GyroBias: %.5f | AccBiasX: %.3f | AccBiasY: %.3f | FailLeft: %.2f s"),
				AHRSGyroBias_radps,
				AHRSAccelBiasX_mps2,
				AHRSAccelBiasY_mps2,
				AHRSFailureTimeRemaining_s
			)
		);
	}
	void AOtter_Pawn::DrawGPSDebug() const
	{
		if (!GEngine)
		{
			return;
		}

		GEngine->AddOnScreenDebugMessage(
			100,
			0.0f,
			FColor::Green,
			FString::Printf(TEXT("REAL -> X: %.2f m | Y: %.2f m"), GPSRealX_m, GPSRealY_m)
		);

		if (bGPSValid)
		{
			GEngine->AddOnScreenDebugMessage(
				101,
				0.0f,
				(GPSHealth == EGPSHealth::Degraded) ? FColor::Orange : FColor::Yellow,
				FString::Printf(
					TEXT("GPS  -> X: %.2f m | Y: %.2f m | Erro: %.2f m | VALID"),
					GPSMeasuredX_m,
					GPSMeasuredY_m,
					GPSError_m
				)
			);
		}
		else
		{
			GEngine->AddOnScreenDebugMessage(
				101,
				0.0f,
				FColor::Red,
				FString::Printf(TEXT("GPS  -> INVALID / SIGNAL LOST"))
			);
		}

		const TCHAR* HealthText = TEXT("Healthy");
		switch (GPSHealth)
		{
		case EGPSHealth::Healthy:  HealthText = TEXT("Healthy"); break;
		case EGPSHealth::Degraded: HealthText = TEXT("Degraded"); break;
		case EGPSHealth::Frozen:   HealthText = TEXT("Frozen"); break;
		case EGPSHealth::Lost:     HealthText = TEXT("Lost"); break;
		default: break;
		}

		GEngine->AddOnScreenDebugMessage(
			102,
			0.0f,
			FColor::Cyan,
			FString::Printf(
				TEXT("GPS  -> Rate: %.1f Hz | Sigma: %.2f m | Lat: %.2f s | Health: %s"),
				GPSUpdateRateHz,
				GPSPosSigma_m,
				GPSLatency_s,
				HealthText
			)
		);

		GEngine->AddOnScreenDebugMessage(
			103,
			0.0f,
			FColor::White,
			FString::Printf(
				TEXT("GPS  -> Pending: %d | BiasX: %.2f | BiasY: %.2f | FailTimeLeft: %.2f s"),
				GPSPendingSamples.Num(),
				GPSBiasX_m,
				GPSBiasY_m,
				GPSFailureTimeRemaining_s
			)
		);
	}
	void AOtter_Pawn::StepDynamics6DOF(float Dt)
	{
		//Massas efetivas 6DOF
		const float m11 = Mass + X_u_dot;
		const float m22 = Mass + Y_v_dot;
		const float m33 = Mass + Z_w_dot;
		const float m44 = InertiaRoll + K_p_dot;
		const float m55 = InertiaPitch + M_q_dot;
		const float m66 = InertiaYaw + N_r_dot;

		if (m11 <= KINDA_SMALL_NUMBER || m22 <= KINDA_SMALL_NUMBER ||
			m33 <= KINDA_SMALL_NUMBER || m44 <= KINDA_SMALL_NUMBER ||
			m55 <= KINDA_SMALL_NUMBER || m66 <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		//Misturador e dinâmica do atuador
		const float MixLeftCmd = FMath::Clamp(InputForward - InputRight, -1.0f, 1.0f);
		const float MixRightCmd = FMath::Clamp(InputForward + InputRight, -1.0f, 1.0f);

		const float TargetCurrentLeft_A = MixLeftCmd * MaxCommandCurrent;
		const float TargetCurrentRight_A = MixRightCmd * MaxCommandCurrent;

		const float TauMotor = FMath::Max(MotorTimeConstant, 0.001f);
		const float Alpha = 1.0f - FMath::Exp(-Dt / TauMotor);

		CurrentLeft_A = FMath::Lerp(CurrentLeft_A, TargetCurrentLeft_A, Alpha);
		CurrentRight_A = FMath::Lerp(CurrentRight_A, TargetCurrentRight_A, Alpha);

		ThrustLeft_N = CalculateT200Thrust(CurrentLeft_A);
		ThrustRight_N = CalculateT200Thrust(CurrentRight_A);

		//Forças/momentos propulsivos com geometria dos thrusters
		const float FxL = ThrustLeft_N;
		const float FxR = ThrustRight_N;

		const FVector rL(ThrusterXOffset_m, -ThrusterHalfBeam, ThrusterZOffset_m);
		const FVector rR(ThrusterXOffset_m, ThrusterHalfBeam, ThrusterZOffset_m);

		const FVector FL(FxL, 0.0f, 0.0f);
		const FVector FR(FxR, 0.0f, 0.0f);

		const FVector ML = FVector::CrossProduct(rL, FL);
		const FVector MR = FVector::CrossProduct(rR, FR);
		const FVector Mtot = ML + MR;

		const float TauPropX = FxL + FxR;
		const float TauPropY = 0.0f;
		const float TauPropZ = 0.0f;

		const float TauPropK = Mtot.X; // roll
		const float TauPropM = -Mtot.Y; // pitch
		const float TauPropN = -Mtot.Z; // yaw

		//Damping e coriolis
		const float TauDampX = -X_u * NuU_mps;
		const float TauDampY = -Y_v * NuV_mps;
		const float TauDampN = -N_r * NuR_radps;

		const float TauCorX = m22 * NuV_mps * NuR_radps;
		const float TauCorY = -m11 * NuU_mps * NuR_radps;
		const float TauCorN = (m11 - m22) * NuU_mps * NuV_mps;

		//Hidrostática + lift simplificado
		const float WeightZ_N = -Mass * GravityZ;
		const float RestoreZ = BuoyancyZ_N + WeightZ_N;

		const float RestoreK = -(Mass * GravityZ) * GM_T * FMath::Sin(EtaPhi_rad);
		const float RestoreM = -(Mass * GravityZ) * GM_L * FMath::Sin(EtaTheta_rad);

		const float TauLiftZ = HeaveLiftCoeff * NuU_mps * FMath::Abs(NuU_mps);
		const float TauLiftM = PitchLiftCoeff * NuU_mps * FMath::Abs(NuU_mps);

		//Soma total das forças/momentos
		const float TauX = TauPropX + TauDampX + TauCorX;
		const float TauY = TauPropY + TauDampY + TauCorY;
		const float TauZ = TauPropZ + TauLiftZ + RestoreZ - Z_w * NuW_mps;

		const float TauK = TauPropK + RestoreK - K_p * NuP_radps;
		const float TauM = TauPropM + TauLiftM + RestoreM - M_q * NuQ_radps;
		const float TauN = TauPropN + TauDampN + TauCorN;

		//nu_dot 6DOF — semi-implicit Euler
		const float Udot = TauX / m11;
		const float Vdot = TauY / m22;
		const float Wdot = TauZ / m33;
		const float Pdot = TauK / m44;
		const float Qdot = TauM / m55;
		const float Rdot = TauN / m66;

		NuU_mps += Udot * Dt;
		NuV_mps += Vdot * Dt;
		NuW_mps += Wdot * Dt;
		NuP_radps += Pdot * Dt;
		NuQ_radps += Qdot * Dt;
		NuR_radps += Rdot * Dt;

		//Cinemática 6DOF — Jacobiano J(φ, θ, ψ) convenção ZYX
		const float Cpsi = FMath::Cos(EtaPsi_rad);
		const float Spsi = FMath::Sin(EtaPsi_rad);
		const float Cphi = FMath::Cos(EtaPhi_rad);
		const float Sphi = FMath::Sin(EtaPhi_rad);
		const float Ctheta = FMath::Cos(EtaTheta_rad);
		const float Stheta = FMath::Sin(EtaTheta_rad);

		const float SafeCtheta = FMath::Max(FMath::Abs(Ctheta), 0.01f);
		const float Ttheta = Stheta / SafeCtheta;

		const float Xdot =
			(Cpsi * Ctheta) * NuU_mps
			+ (Cpsi * Stheta * Sphi - Spsi * Cphi) * NuV_mps
			+ (Cpsi * Stheta * Cphi + Spsi * Sphi) * NuW_mps;

		const float Ydot =
			(Spsi * Ctheta) * NuU_mps
			+ (Spsi * Stheta * Sphi + Cpsi * Cphi) * NuV_mps
			+ (Spsi * Stheta * Cphi - Cpsi * Sphi) * NuW_mps;

		const float Zdot =
			(-Stheta) * NuU_mps
			+ (Ctheta * Sphi) * NuV_mps
			+ (Ctheta * Cphi) * NuW_mps;

		const float PhiDot =
			NuP_radps + Sphi * Ttheta * NuQ_radps + Cphi * Ttheta * NuR_radps;

		const float ThetaDot =
			Cphi * NuQ_radps - Sphi * NuR_radps;

		const float PsiDot =
			(Sphi / SafeCtheta) * NuQ_radps + (Cphi / SafeCtheta) * NuR_radps;

		EtaX_m += Xdot * Dt;
		EtaY_m += Ydot * Dt;
		EtaZ_m += Zdot * Dt;
		EtaPhi_rad += PhiDot * Dt;
		EtaTheta_rad += ThetaDot * Dt;
		EtaPsi_rad += PsiDot * Dt;

		EtaPhi_rad = FMath::UnwindRadians(EtaPhi_rad);
		EtaTheta_rad = FMath::UnwindRadians(EtaTheta_rad);
		EtaPsi_rad = FMath::UnwindRadians(EtaPsi_rad);

		BodySpecificForceX_mps2 = Udot - NuR_radps * NuV_mps + NuQ_radps * NuW_mps;
		BodySpecificForceY_mps2 = Vdot + NuR_radps * NuU_mps - NuP_radps * NuW_mps;
		YawAccel_radps2 = Rdot;
	}