// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Sensors/MjLidarTypes.h"
#include "Math/RandomStream.h"
#include <atomic>
#include "MjLidarSensor.generated.h"

/**
 * @class UMjLidarSensor
 * @brief Unreal-side raycast lidar that measures the MuJoCo scene directly.
 *
 * Unlike UMjSensor subclasses (e.g. the rangefinder, which is a MuJoCo
 * passthrough), this component does not register anything into the MuJoCo
 * spec: no ExportTo / RegisterToSpec. It is a pure UE-side observer, in the
 * spirit of UMjCamera.
 *
 * Data path (follows docs/concepts/architecture.md):
 *  1. The physics engine's post-step callback (physics thread, inside
 *     CallbackMutex) reads the mount body/site pose from mjData, builds the
 *     beam grid in MuJoCo coordinates, and casts all rays with mj_multiRay.
 *  2. The raw scan (origin, sensor rotation, per-beam ranges/geom ids) is
 *     copied into a mutex-guarded buffer shared with the game thread. The
 *     game thread never touches mjData.
 *  3. TickComponent (game thread) drains the latest scan, converts hits to
 *     Unreal world coordinates with MjUtils::MjToUEPosition, aggregates
 *     targets, draws debug output, and broadcasts OnLidarScan.
 *
 * Mount resolution: the nearest UMjBody or UMjSite ancestor in the component
 * attachment chain is used as the MuJoCo pose source (resolved once per Bind
 * via the ancestor's spec element id, published to the physics thread under
 * CallbackMutex). With no MuJoCo ancestor the sensor falls back to a static
 * world mount using the component's world pose captured at resolve time.
 *
 * Beam convention (MuJoCo coordinates): azimuth sweeps around the sensor's
 * local +Z (up), elevation is positive above the local X-XY plane, and
 * azimuth/elevation of 0/0 points along the sensor's local +X (the Unreal
 * component's forward axis after the UE->MuJoCo handedness conversion).
 * The full-circle azimuth grid is endpoint-exclusive; partial grids are
 * endpoint-inclusive; a single beam sits at the midpoint of its range.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjLidarSensor : public UMjComponent
{
	GENERATED_BODY()

public:
	UMjLidarSensor();

	// --- Scan pattern ---

	/** Number of azimuth beams (columns). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 AzimuthBeams = 360;

	/** Azimuth range start, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "-360", ClampMax = "360"))
	float AzimuthFovStart = -180.0f;

	/** Azimuth range end, degrees. A span of 360 sweeps the full circle (endpoint-exclusive). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "-360", ClampMax = "360"))
	float AzimuthFovEnd = 180.0f;

	/** Number of elevation beams (rows). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 ElevationBeams = 1;

	/** Elevation range start, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "-90", ClampMax = "90"))
	float ElevationFovStart = 0.0f;

	/** Elevation range end, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "-90", ClampMax = "90"))
	float ElevationFovEnd = 0.0f;

	/** Minimum range in metres; returns closer than this are discarded (blind zone). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "0", ClampMax = "50"))
	float MinRange = 0.1f;

	/** Maximum range in metres; also the mj_multiRay cutoff. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Scan", meta = (ClampMin = "0.1", ClampMax = "1000"))
	float MaxRange = 100.0f;

	// --- Timing ---

	/**
	 * Scans per second of simulation time (rate-limited inside the post-step
	 * callback, so the lidar does not rescan on every physics step).
	 * Values <= 0 scan on every physics step.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Timing", meta = (ClampMin = "0"))
	float ScanFrequencyHz = 10.0f;

	// --- Geometry ---

	/**
	 * Bitmask over MuJoCo geom groups 0..5 (bit g enables group g; bit 0 has
	 * value 1, bit 5 has value 32). URLab convention: group 3 = collision-only
	 * geoms, group 2 = visual-only meshes; quick-converted simple primitives
	 * and MJCF-imported geoms default to group 0. Default: group 3 only (8).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry", meta = (ClampMin = "0", ClampMax = "63"))
	int32 GeomGroupMask = 8;

	/** Include static (worldbody) geoms such as the floor. Leave enabled for environment sensing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry")
	bool bHitStaticGeoms = true;

	/** Exclude the mount body's own geoms from raycasts (avoids self-hits at the origin). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry")
	bool bExcludeMountBody = true;

	// --- Noise ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Noise")
	bool bEnableNoise = false;

	/** Range noise standard deviation, metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float RangeNoiseStdDev = 0.01f;

	/** Azimuth noise standard deviation, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float AzimuthNoiseStdDev = 0.0f;

	/** Elevation noise standard deviation, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float ElevationNoiseStdDev = 0.0f;

	// --- Output ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Output")
	EMjLidarOutputMode OutputMode = EMjLidarOutputMode::Points;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Output")
	bool bDrawDebugPoints = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Output")
	bool bDrawDebugLines = false;

	// --- Target aggregation ---

	/**
	 * Maximum distance between two hit points (metres) for them to join the
	 * same cluster. Also gates target matching between consecutive scans.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Targeting", meta = (ClampMin = "0.01"))
	float ClusterDistanceThreshold = 0.5f;

	/**
	 * Compute per-target radial speed from centroid differencing. Kinematic
	 * approximation, not a Doppler measurement.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Targeting")
	bool bComputeVelocity = true;

	/**
	 * Number of past centroids kept per target (velocity smoothing window
	 * and coasting horizon in scans).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Targeting", meta = (ClampMin = "2", ClampMax = "64"))
	int32 HistorySize = 8;

	// --- Results ---

	/** Hit points of the most recently consumed scan (empty in Targets mode). */
	UPROPERTY(BlueprintReadOnly, Category = "MuJoCo|Lidar")
	TArray<FMjLidarPoint> LastPoints;

	/** Aggregated targets of the most recently consumed scan (empty in Points mode). */
	UPROPERTY(BlueprintReadOnly, Category = "MuJoCo|Lidar")
	TArray<FMjLidarTarget> LastTargets;

	/** Broadcast on the game thread after each scan is consumed. */
	UPROPERTY(BlueprintAssignable, Category = "MuJoCo|Lidar")
	FOnLidarScan OnLidarScan;

	/** Returns the most recently consumed scan (points/targets per OutputMode plus metadata). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Lidar")
	FMjLidarScan GetLastScan() const;

	// --- Pure logic helpers (static, unit-testable) ---

	/**
	 * Expands the beam angle grid. Full-circle azimuth spans (|end-start| >=
	 * 359.99) are endpoint-exclusive (periodic); partial spans and elevation
	 * are endpoint-inclusive; a single beam sits at the span midpoint.
	 */
	static void BuildBeamAngles(int32 AzBeams, float AzStart, float AzEnd,
		int32 ElBeams, float ElStart, float ElEnd,
		TArray<float>& OutAzimuths, TArray<float>& OutElevations);

	/**
	 * Unit beam direction for the given angles, expressed with MuJoCo axis
	 * conventions as an FVector container (azimuth around +Z from +X,
	 * elevation positive towards +Z).
	 */
	static FVector SphericalDirectionMj(float AzimuthDeg, float ElevationDeg);

	/**
	 * Clusters points by Euclidean distance using a spatial hash + union-find.
	 * OutClusterIds[i] is the cluster index of Points[i]; OutClusterSizes[k]
	 * is the size of cluster k.
	 */
	static void ClusterPoints(const TArray<FVector>& Points, float ThresholdCm,
		TArray<int32>& OutClusterIds, TArray<int32>& OutClusterSizes);

	/** Zero-mean Gaussian sample drawn from the stream (Box-Muller). */
	static float SampleGaussian(FRandomStream& Stream, float StdDev);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void Bind(mjModel* model, mjData* data, const FString& Prefix = TEXT("")) override;

private:
	// ---- Mount (game thread resolves, physics thread reads; guarded by
	// UMjPhysicsEngine::CallbackMutex, which the post-step callback always
	// holds and the game thread takes briefly while publishing) ----
	struct FMountState
	{
		int32 Type = 0;      // 0 = none/unresolved, 1 = body, 2 = site
		int32 Id = -1;       // body id or site id in the compiled model
		mjModel* Model = nullptr; // model the ids were resolved against
		double RelPosMj[3] = { 0, 0, 0 };  // sensor position in mount frame, MuJoCo metres
		double RelQuatMj[4] = { 1, 0, 0, 0 }; // sensor rotation in mount frame, MuJoCo (w,x,y,z)
	};
	FMountState m_Mount;

	// Set in EndPlay so the callback stops scanning even while the object is
	// still alive; also guards against destruction without EndPlay when read
	// together with the weak pointer.
	std::atomic<bool> m_bStopped{ false };

	// ---- Scan handoff (physics writes, game thread copies; m_DataMutex) ----
	struct FRawScan
	{
		double OriginMj[3] = { 0, 0, 0 };   // sensor origin, MuJoCo metres
		double SensorMatMj[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // row-major sensor rotation
		TArray<float> Azimuths;   // AzN entries
		TArray<float> Elevations; // ElN entries
		TArray<double> Ranges;    // NumRays entries, -1 = miss
		TArray<int32> GeomIds;    // NumRays entries, -1 = miss
		double SimTime = 0.0;
		int64 ScanId = -1;
		int32 AzCount = 0;
		int32 ElCount = 0;
	};
	FCriticalSection m_DataMutex;
	FRawScan m_SharedScan;
	FRawScan m_LocalScan;
	int64 m_LastConsumedScanId = -1;
	int64 m_LastScanId = -1;
	double m_LastScanSimTime = 0.0;

	// ---- Physics-thread-only state (no locking needed) ----
	int64 m_ScanCounter = 0;
	double m_NextScanTime = 0.0;
	TArray<float> m_ScratchAz;
	TArray<float> m_ScratchEl;
	TArray<double> m_ScratchVec;   // 3 * NumRays
	TArray<double> m_ScratchDist;  // NumRays
	TArray<int32> m_ScratchGeomId; // NumRays
	FRandomStream m_NoiseStream;

	// ---- Game-thread-only state ----
	bool m_bMountResolved = false;
	bool m_bCallbackRegistered = false;

	/** Target tracking state (game thread). */
	struct FTargetTrack
	{
		int32 Id = -1;
		FVector Centroid = FVector::ZeroVector;
		double SimTime = 0.0;
		int32 Misses = 0;
		TArray<FVector> HistoryPos; // bounded by HistorySize
		TArray<double> HistoryTime;
	};
	TArray<FTargetTrack> m_Tracks;
	int32 m_NextTargetId = 1;

	// ---- Internals ----

	/** Resolves the mount body/site id and relative pose; publishes under CallbackMutex. */
	void ResolveMount();

	/** Runs inside the physics engine's post-step callback (CallbackMutex held). */
	void PhysicsPostStep(mjModel* m, mjData* d);

	/** Copies out and post-processes the newest scan (game thread). */
	void ConsumeScan();

	/** Clusters current hits into targets, matching against m_Tracks. */
	void BuildTargets(const TArray<FVector>& HitPositionsCm, double SimTime, TArray<FMjLidarTarget>& OutTargets);
};
