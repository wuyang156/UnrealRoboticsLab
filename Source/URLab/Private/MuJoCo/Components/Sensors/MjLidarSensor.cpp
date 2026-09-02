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

#include "MuJoCo/Components/Sensors/MjLidarSensor.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "DrawDebugHelpers.h"

UMjLidarSensor::UMjLidarSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjLidarSensor::BeginPlay()
{
	Super::BeginPlay();
}

void UMjLidarSensor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	m_bStopped.store(true, std::memory_order_release);

	// Take the callback mutex briefly so any in-flight physics callback is
	// finished (and none can start) when we return from EndPlay.
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			FScopeLock Lock(&Manager->PhysicsEngine->CallbackMutex);
		}
	}

	SetComponentTickEnabled(false);
	Super::EndPlay(EndPlayReason);
}

void UMjLidarSensor::Bind(mjModel* model, mjData* data, const FString& Prefix)
{
	Super::Bind(model, data, Prefix);

	// New model compiled: drop the resolved mount so the next Tick resolves
	// against the fresh model. The physics callback keeps validating
	// m_Mount.Model against the mjModel it receives, so a stale mount can
	// never be used against the new model.
	m_bMountResolved = false;
}

void UMjLidarSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AAMjManager* Manager = AAMjManager::GetManager();
	if (!Manager || !Manager->PhysicsEngine)
	{
		return;
	}

	// 1. Resolve the mount once per Bind (game thread, publishes under CallbackMutex).
	if (!m_bMountResolved && m_Model)
	{
		ResolveMount();
	}

	// 2. Register the physics-thread callback once (weak guard: it becomes a
	// no-op once this component is gone, matching the manager's own pattern).
	if (!m_bCallbackRegistered)
	{
		TWeakObjectPtr<UMjLidarSensor> WeakThis(this);
		Manager->PhysicsEngine->RegisterPostStepCallback(
			[WeakThis](mjModel* m, mjData* d) {
				UMjLidarSensor* Self = WeakThis.Get();
				if (!Self || Self->m_bStopped.load(std::memory_order_acquire))
				{
					return;
				}
				Self->PhysicsPostStep(m, d);
			});
		m_bCallbackRegistered = true;
	}

	// 3. Drain the newest scan.
	ConsumeScan();
}

void UMjLidarSensor::ResolveMount()
{
	// Walk the attachment chain to the nearest MuJoCo pose source.
	USceneComponent* MountComponent = nullptr;
	{
		USceneComponent* Parent = GetAttachParent();
		while (Parent)
		{
			if (Parent->IsA<UMjSite>() || Parent->IsA<UMjBody>())
			{
				MountComponent = Parent;
				break;
			}
			Parent = Parent->GetAttachParent();
		}
	}

	// Compose the mount-relative transform of this sensor, walking from this
	// component up to (but excluding) the mount component. World_child =
	// World_parent * Rel_child, so Rel_mount_sensor = Rel(...*Rel(this)) with
	// each parent's relative transform pre-multiplied.
	FTransform Rel = GetRelativeTransform();
	if (MountComponent)
	{
		USceneComponent* Ancestor = GetAttachParent();
		while (Ancestor && Ancestor != MountComponent)
		{
			Rel = Ancestor->GetRelativeTransform() * Rel;
			Ancestor = Ancestor->GetAttachParent();
		}
	}

	int32 MountType = 0;
	int32 MountId = -1;

	UMjComponent* MountMjComp = Cast<UMjComponent>(MountComponent);
	if (MountMjComp)
	{
		int32 Candidate = -1;
		if (const UMjBody* Body = Cast<UMjBody>(MountMjComp))
		{
			Candidate = Body->GetMj().id;
			if (Candidate < 0)
			{
				if (mjsElement* Elem = Body->GetSpecElementForDiagnostics())
				{
					Candidate = mjs_getId(Elem);
				}
			}
			if (Candidate >= 0 && m_Model && Candidate < m_Model->nbody)
			{
				MountType = 1;
				MountId = Candidate;
			}
		}
		else if (const UMjSite* Site = Cast<UMjSite>(MountMjComp))
		{
			Candidate = Site->GetMj().id;
			if (Candidate < 0)
			{
				if (mjsElement* Elem = Site->GetSpecElementForDiagnostics())
				{
					Candidate = mjs_getId(Elem);
				}
			}
			if (Candidate >= 0 && m_Model && Candidate < m_Model->nsite)
			{
				MountType = 2;
				MountId = Candidate;
			}
		}

		if (MountType == 0)
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[MjLidarSensor] '%s': mount component '%s' has no valid MuJoCo id; falling back to a static world mount."),
				*GetName(), *MountComponent->GetName());
		}
	}

	if (MountType == 0)
	{
		// No usable MuJoCo ancestor: static world mount. The sensor pose is
		// captured once (worldbody never moves).
		Rel = GetComponentTransform();
		MountType = 1;
		MountId = 0; // worldbody
		UE_LOG(LogURLab, Log,
			TEXT("[MjLidarSensor] '%s': no UMjBody/UMjSite ancestor; using static world mount. Attach under a body to follow the robot."),
			*GetName());
	}

	// Convert the relative pose into MuJoCo coordinates (metres, w-first quat).
	double RelPosMj[3];
	double RelQuatMj[4];
	MjUtils::UEToMjPosition(Rel.GetLocation(), RelPosMj);
	MjUtils::UEToMjRotation(Rel.GetRotation(), RelQuatMj);

	// Publish under CallbackMutex so the physics thread (which reads m_Mount
	// while holding it) never observes a torn state.
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			FScopeLock Lock(&Manager->PhysicsEngine->CallbackMutex);
			m_Mount.Type = MountType;
			m_Mount.Id = MountId;
			m_Mount.Model = m_Model;
			for (int32 i = 0; i < 3; ++i)
			{
				m_Mount.RelPosMj[i] = RelPosMj[i];
			}
			for (int32 i = 0; i < 4; ++i)
			{
				m_Mount.RelQuatMj[i] = RelQuatMj[i];
			}
			m_bMountResolved = true;
			return;
		}
	}

	// Manager went away mid-resolve: retry next Tick.
	m_bMountResolved = false;
}

void UMjLidarSensor::PhysicsPostStep(mjModel* m, mjData* d)
{
	if (!m || !d)
	{
		return;
	}

	// Snapshot the configuration first so one scan is internally consistent
	// even if the game thread edits properties concurrently (single-word
	// reads are atomic on our target platforms).
	const int32 AzN = FMath::Clamp(AzimuthBeams, 1, 4096);
	const int32 ElN = FMath::Clamp(ElevationBeams, 1, 4096);
	const int64 NumRays64 = static_cast<int64>(AzN) * ElN;
	if (NumRays64 <= 0 || NumRays64 > 16 * 1024 * 1024)
	{
		return;
	}
	const int32 NumRays = static_cast<int32>(NumRays64);
	const float AzStart = AzimuthFovStart;
	const float AzEnd = AzimuthFovEnd;
	const float ElStart = ElevationFovStart;
	const float ElEnd = ElevationFovEnd;
	const float MinR = FMath::Max(MinRange, 0.0f);
	const float MaxR = FMath::Max(MaxRange, MinR + 0.01f);
	const float Hz = ScanFrequencyHz;

	// Rate-limit on simulation time (not wall clock), so the sensor honours
	// sim-speed changes and pauses for free.
	if (Hz > 0.0f)
	{
		if (d->time + 1e-9 < m_NextScanTime)
		{
			return;
		}
		// Simulation time jumped backwards (reset): resync.
		if (m_NextScanTime > d->time + 1.0)
		{
			m_NextScanTime = d->time;
		}
		m_NextScanTime = FMath::Max(m_NextScanTime + 1.0 / (double)Hz, d->time);
	}

	// Mount must be resolved against this exact model.
	if (m_Mount.Model != m || m_Mount.Type == 0)
	{
		return;
	}

	const double* MountXpos = nullptr;
	const double* MountXmat = nullptr;
	int32 BodyExclude = -1;
	if (m_Mount.Type == 1)
	{
		if (m_Mount.Id < 0 || m_Mount.Id >= m->nbody)
		{
			return;
		}
		MountXpos = d->xpos + 3 * m_Mount.Id;
		MountXmat = d->xmat + 9 * m_Mount.Id;
		// Never exclude the worldbody (id 0): it owns the floor and other
		// static environment, which a lidar must see.
		if (bExcludeMountBody && m_Mount.Id > 0)
		{
			BodyExclude = m_Mount.Id;
		}
	}
	else
	{
		if (m_Mount.Id < 0 || m_Mount.Id >= m->nsite)
		{
			return;
		}
		MountXpos = d->site_xpos + 3 * m_Mount.Id;
		MountXmat = d->site_xmat + 9 * m_Mount.Id;
		if (bExcludeMountBody)
		{
			const int32 SiteBody = m->site_bodyid[m_Mount.Id];
			if (SiteBody > 0)
			{
				BodyExclude = SiteBody;
			}
		}
	}

	// Sensor frame in MuJoCo world coordinates: R_sensor = R_mount * R_rel,
	// p_sensor = p_mount + R_mount * p_rel.
	double RelMat[9];
	mju_quat2Mat(RelMat, m_Mount.RelQuatMj);
	double SensorMat[9];
	mju_mulMatMat(SensorMat, MountXmat, RelMat, 3, 3, 3);
	double SensorPos[3];
	double RelPosWorld[3];
	mju_mulMatVec3(RelPosWorld, MountXmat, m_Mount.RelPosMj);
	SensorPos[0] = MountXpos[0] + RelPosWorld[0];
	SensorPos[1] = MountXpos[1] + RelPosWorld[1];
	SensorPos[2] = MountXpos[2] + RelPosWorld[2];

	// Beam angles (+ angular noise).
	BuildBeamAngles(AzN, AzStart, AzEnd, ElN, ElStart, ElEnd, m_ScratchAz, m_ScratchEl);
	const bool bNoise = bEnableNoise;
	const float AzNoise = bNoise ? FMath::Max(AzimuthNoiseStdDev, 0.0f) : 0.0f;
	const float ElNoise = bNoise ? FMath::Max(ElevationNoiseStdDev, 0.0f) : 0.0f;
	const float RangeNoise = bNoise ? FMath::Max(RangeNoiseStdDev, 0.0f) : 0.0f;
	if (AzNoise > 0.0f)
	{
		for (int32 a = 0; a < AzN; ++a)
		{
			m_ScratchAz[a] += SampleGaussian(m_NoiseStream, AzNoise);
		}
	}
	if (ElNoise > 0.0f)
	{
		for (int32 e = 0; e < ElN; ++e)
		{
			m_ScratchEl[e] += SampleGaussian(m_NoiseStream, ElNoise);
		}
	}

	// Direction fan: ray index = e * AzN + a (elevation-major).
	m_ScratchVec.SetNumUninitialized(3 * NumRays);
	{
		int32 Idx = 0;
		for (int32 e = 0; e < ElN; ++e)
		{
			for (int32 a = 0; a < AzN; ++a)
			{
				const FVector Dir = SphericalDirectionMj(m_ScratchAz[a], m_ScratchEl[e]);
				const double V[3] = { (double)Dir.X, (double)Dir.Y, (double)Dir.Z };
				mju_mulMatVec3(m_ScratchVec.GetData() + 3 * Idx, SensorMat, V);
				++Idx;
			}
		}
	}

	// Geom group filter: bit g of the mask enables group g. All-groups-set is
	// equivalent to no filter at all (NULL), which skips group exclusion.
	mjtByte GroupBytes[mjNGROUP];
	bool bAllGroups = true;
	const int32 Mask = GeomGroupMask & 0x3F;
	for (int32 g = 0; g < mjNGROUP; ++g)
	{
		GroupBytes[g] = ((Mask >> g) & 1) ? 1 : 0;
		bAllGroups = bAllGroups && GroupBytes[g] != 0;
	}

	// Cast all rays from the single sensor origin.
	m_ScratchDist.SetNumUninitialized(NumRays);
	m_ScratchGeomId.SetNumUninitialized(NumRays);
	mj_multiRay(m, d, SensorPos, m_ScratchVec.GetData(),
		bAllGroups ? nullptr : GroupBytes,
		bHitStaticGeoms ? 1 : 0,
		BodyExclude,
		m_ScratchGeomId.GetData(), m_ScratchDist.GetData(), nullptr,
		NumRays, (mjtNum)MaxR);

	// Post-filter: blind zone, range noise.
	for (int32 i = 0; i < NumRays; ++i)
	{
		double& R = m_ScratchDist[i];
		if (R >= 0.0)
		{
			if (R < (double)MinR)
			{
				R = -1.0; // inside the blind zone: treat as a miss
				m_ScratchGeomId[i] = -1;
			}
			else if (RangeNoise > 0.0f)
			{
				R = FMath::Max(R + (double)SampleGaussian(m_NoiseStream, RangeNoise), 0.0);
			}
		}
		else
		{
			m_ScratchGeomId[i] = -1;
		}
	}

	// Publish the raw scan for the game thread.
	{
		FScopeLock Lock(&m_DataMutex);
		m_SharedScan.ScanId = ++m_ScanCounter;
		m_SharedScan.SimTime = d->time;
		m_SharedScan.AzCount = AzN;
		m_SharedScan.ElCount = ElN;
		for (int32 i = 0; i < 3; ++i)
		{
			m_SharedScan.OriginMj[i] = SensorPos[i];
		}
		for (int32 i = 0; i < 9; ++i)
		{
			m_SharedScan.SensorMatMj[i] = SensorMat[i];
		}
		m_SharedScan.Azimuths = m_ScratchAz;
		m_SharedScan.Elevations = m_ScratchEl;
		m_SharedScan.Ranges = m_ScratchDist;
		m_SharedScan.GeomIds = m_ScratchGeomId;
	}
}

void UMjLidarSensor::ConsumeScan()
{
	{
		FScopeLock Lock(&m_DataMutex);
		if (m_SharedScan.ScanId < 0 || m_SharedScan.ScanId == m_LastConsumedScanId)
		{
			return;
		}
		// Copy out (keeps the shared buffers' capacity stable so the physics
		// thread never re-allocates under the lock either).
		m_LocalScan = m_SharedScan;
		m_LastConsumedScanId = m_SharedScan.ScanId;
	}

	const int64 ScanId = m_LocalScan.ScanId;
	const double SimTime = m_LocalScan.SimTime;
	const int32 AzN = m_LocalScan.AzCount > 0 ? m_LocalScan.AzCount : 1;
	const int32 NumRays = m_LocalScan.Ranges.Num();
	const bool bWantPoints = OutputMode == EMjLidarOutputMode::Points || OutputMode == EMjLidarOutputMode::Both;
	const bool bWantTargets = OutputMode == EMjLidarOutputMode::Targets || OutputMode == EMjLidarOutputMode::Both;

	// Sensor origin in Unreal world coordinates.
	const double OriginD[3] = { m_LocalScan.OriginMj[0], m_LocalScan.OriginMj[1], m_LocalScan.OriginMj[2] };
	const FVector SensorPosUe = MjUtils::MjToUEPosition(m_LocalScan.OriginMj);

	TArray<FMjLidarPoint> Points;
	TArray<FVector> HitPositionsCm;
	TArray<int32> HitIndices; // beam index of each hit (parallel to HitPositionsCm)
	Points.Reserve(bWantPoints ? NumRays : 0);
	HitPositionsCm.Reserve(NumRays);
	HitIndices.Reserve(NumRays);

	for (int32 i = 0; i < NumRays; ++i)
	{
		const double R = m_LocalScan.Ranges[i];
		if (R < 0.0)
		{
			continue; // miss
		}
		const int32 a = i % AzN;
		const int32 e = i / AzN;
		const float Az = m_LocalScan.Azimuths.IsValidIndex(a) ? m_LocalScan.Azimuths[a] : 0.0f;
		const float El = m_LocalScan.Elevations.IsValidIndex(e) ? m_LocalScan.Elevations[e] : 0.0f;

		// Hit point in MuJoCo world space: origin + direction * range.
		const FVector Dir = SphericalDirectionMj(Az, El);
		const double DirD[3] = { (double)Dir.X, (double)Dir.Y, (double)Dir.Z };
		double Rotated[3];
		mju_mulMatVec3(Rotated, m_LocalScan.SensorMatMj, DirD);
		const double HitMj[3] = {
			OriginD[0] + Rotated[0] * R,
			OriginD[1] + Rotated[1] * R,
			OriginD[2] + Rotated[2] * R
		};

		HitPositionsCm.Add(MjUtils::MjToUEPosition(HitMj));
		HitIndices.Add(i);

		if (bWantPoints)
		{
			FMjLidarPoint& P = Points.AddDefaulted_GetRef();
			P.WorldPos = HitPositionsCm.Last();
			P.AzimuthDeg = Az;
			P.ElevationDeg = El;
			P.RangeM = (float)R;
			P.HitGeomId = m_LocalScan.GeomIds.IsValidIndex(i) ? m_LocalScan.GeomIds[i] : -1;
			P.FrameId = ScanId;
		}
	}

	// Debug drawing (game thread, world coordinates).
	if (bDrawDebugPoints || bDrawDebugLines)
	{
		UWorld* World = GetWorld();
		if (World)
		{
			const float Lifetime = FMath::Clamp(ScanFrequencyHz > 0.0f ? 1.0f / ScanFrequencyHz : 0.05f, 0.05f, 0.5f);
			for (int32 h = 0; h < HitPositionsCm.Num(); ++h)
			{
				if (bDrawDebugPoints)
				{
					DrawDebugPoint(World, HitPositionsCm[h], 4.0f, FColor::Green, false, Lifetime);
				}
				if (bDrawDebugLines)
				{
					DrawDebugLine(World, SensorPosUe, HitPositionsCm[h], FColor::Silver, false, Lifetime, 0, 0.15f);
				}
			}
		}
	}

	LastPoints = MoveTemp(Points);
	TArray<FMjLidarTarget> Targets;
	if (bWantTargets)
	{
		BuildTargets(HitPositionsCm, SimTime, Targets);
	}
	LastTargets = MoveTemp(Targets);

	m_LastScanId = ScanId;
	m_LastScanSimTime = SimTime;

	// Broadcast the completed scan.
	FMjLidarScan Scan;
	Scan.Points = LastPoints;
	Scan.Targets = LastTargets;
	Scan.ScanId = ScanId;
	Scan.SimTime = SimTime;
	OnLidarScan.Broadcast(Scan);
}

void UMjLidarSensor::BuildTargets(const TArray<FVector>& HitPositionsCm, double SimTime, TArray<FMjLidarTarget>& OutTargets)
{
	OutTargets.Reset();

	TArray<int32> ClusterIds;
	TArray<int32> ClusterSizes;
	ClusterPoints(HitPositionsCm, ClusterDistanceThreshold * 100.0f, ClusterIds, ClusterSizes);

	const int32 NumClusters = ClusterSizes.Num();
	if (NumClusters == 0)
	{
		return;
	}

	// Centroids.
	TArray<FVector> Centroids;
	Centroids.SetNum(NumClusters);
	{
		TArray<int32> Counts;
		Counts.SetNum(NumClusters);
		for (int32 i = 0; i < Centroids.Num(); ++i)
		{
			Centroids[i] = FVector::ZeroVector;
			Counts[i] = 0;
		}
		for (int32 i = 0; i < HitPositionsCm.Num(); ++i)
		{
			const int32 C = ClusterIds[i];
			if (C >= 0 && C < NumClusters)
			{
				Centroids[C] += HitPositionsCm[i];
				++Counts[C];
			}
		}
		for (int32 c = 0; c < NumClusters; ++c)
		{
			if (Counts[c] > 0)
			{
				Centroids[c] /= (float)Counts[c];
			}
		}
	}

	// Sensor position for relative positions.
	const double OriginD[3] = { m_LocalScan.OriginMj[0], m_LocalScan.OriginMj[1], m_LocalScan.OriginMj[2] };
	const FVector SensorPosUe = MjUtils::MjToUEPosition(OriginD);

	// Match clusters to existing tracks (greedy nearest within the gate).
	const float GateCm = FMath::Max(ClusterDistanceThreshold * 300.0f, 1.0f);
	const int32 TrackCount = m_Tracks.Num();
	TArray<int32> TrackOfCluster;
	TrackOfCluster.Init(-1, NumClusters);
	TArray<bool> TrackTaken;
	TrackTaken.Init(false, TrackCount);

	for (int32 Pass = 0; Pass < 2 && TrackCount > 0; ++Pass)
	{
		// Pass 0: pick best (closest) track per cluster; Pass 1: allow the
		// remaining clusters to claim any still-free track (first fit).
		for (int32 c = 0; c < NumClusters; ++c)
		{
			if (TrackOfCluster[c] >= 0)
			{
				continue;
			}
			int32 Best = -1;
			float BestDist = FLT_MAX;
			for (int32 t = 0; t < TrackCount; ++t)
			{
				if (TrackTaken[t])
				{
					continue;
				}
				const float Dist = FVector::Dist(Centroids[c], m_Tracks[t].Centroid);
				if (Dist <= GateCm && Dist < BestDist)
				{
					Best = t;
					BestDist = Dist;
				}
			}
			if (Best >= 0)
			{
				TrackOfCluster[c] = Best;
				TrackTaken[Best] = true;
			}
		}
	}

	const int32 HistCap = FMath::Max(HistorySize, 2);
	for (int32 c = 0; c < NumClusters; ++c)
	{
		int32 TrackIndex = TrackOfCluster[c];
		if (TrackIndex < 0)
		{
			// New track.
			FTargetTrack NewTrack;
			NewTrack.Id = m_NextTargetId++;
			NewTrack.Centroid = Centroids[c];
			NewTrack.SimTime = SimTime;
			NewTrack.Misses = 0;
			NewTrack.HistoryPos.Add(Centroids[c]);
			NewTrack.HistoryTime.Add(SimTime);
			TrackIndex = m_Tracks.Add(MoveTemp(NewTrack));
		}
		else
		{
			FTargetTrack& Track = m_Tracks[TrackIndex];
			Track.Centroid = Centroids[c];
			Track.SimTime = SimTime;
			Track.Misses = 0;
			Track.HistoryPos.Add(Centroids[c]);
			Track.HistoryTime.Add(SimTime);
			while (Track.HistoryPos.Num() > HistCap)
			{
				Track.HistoryPos.RemoveAt(0);
				Track.HistoryTime.RemoveAt(0);
			}
		}

		const FTargetTrack& Track = m_Tracks[TrackIndex];

		FMjLidarTarget& T = OutTargets.AddDefaulted_GetRef();
		T.TargetId = Track.Id;
		T.NumPoints = ClusterSizes[c];
		T.CentroidWorldPos = Track.Centroid;
		T.RelativePos = Track.Centroid - SensorPosUe;

		if (bComputeVelocity && Track.HistoryPos.Num() >= 2)
		{
			const int32 Last = Track.HistoryPos.Num() - 1;
			const double Dt = Track.HistoryTime[Last] - Track.HistoryTime[Last - 1];
			if (Dt > 1e-6)
			{
				// Kinematic approximation: centroid differencing, NOT Doppler.
				const FVector VelCmS = (Track.HistoryPos[Last] - Track.HistoryPos[Last - 1]) / (float)Dt;
				const FVector LoS = T.RelativePos.GetSafeNormal();
				T.RadialSpeed = (float)(FVector::DotProduct(VelCmS, LoS) / 100.0); // cm/s -> m/s
			}
		}
	}

	// Age and prune unmatched tracks. TrackTaken covers the tracks that
	// existed at match time; tracks appended during this scan (indices >=
	// TrackCount) were just matched by construction.
	for (int32 t = m_Tracks.Num() - 1; t >= 0; --t)
	{
		const bool bMatchedThisScan = (t < TrackCount) && TrackTaken[t];
		if (!bMatchedThisScan)
		{
			FTargetTrack& Track = m_Tracks[t];
			++Track.Misses;
			if (Track.Misses > HistCap)
			{
				m_Tracks.RemoveAt(t);
			}
		}
	}
}

FMjLidarScan UMjLidarSensor::GetLastScan() const
{
	FMjLidarScan Scan;
	Scan.Points = LastPoints;
	Scan.Targets = LastTargets;
	Scan.ScanId = m_LastScanId;
	Scan.SimTime = m_LastScanSimTime;
	return Scan;
}

// ---------------------------------------------------------------------------
// Pure logic helpers (static; exercised by URLabEditor automation tests)
// ---------------------------------------------------------------------------

void UMjLidarSensor::BuildBeamAngles(int32 AzBeams, float AzStart, float AzEnd,
	int32 ElBeams, float ElStart, float ElEnd,
	TArray<float>& OutAzimuths, TArray<float>& OutElevations)
{
	const int32 AzN = FMath::Max(AzBeams, 1);
	const int32 ElN = FMath::Max(ElBeams, 1);
	OutAzimuths.SetNum(AzN);
	OutElevations.SetNum(ElN);

	const float AzSpan = AzEnd - AzStart;
	// A full circle wraps onto itself, so it must be endpoint-exclusive;
	// partial spans include both endpoints.
	const bool bPeriodicAz = FMath::Abs(AzSpan) >= 359.99f;
	for (int32 a = 0; a < AzN; ++a)
	{
		if (AzN == 1)
		{
			OutAzimuths[a] = 0.5f * (AzStart + AzEnd);
		}
		else if (bPeriodicAz)
		{
			OutAzimuths[a] = AzStart + AzSpan * ((float)a / (float)AzN);
		}
		else
		{
			OutAzimuths[a] = AzStart + AzSpan * ((float)a / (float)(AzN - 1));
		}
	}

	const float ElSpan = ElEnd - ElStart;
	for (int32 e = 0; e < ElN; ++e)
	{
		if (ElN == 1)
		{
			OutElevations[e] = 0.5f * (ElStart + ElEnd);
		}
		else
		{
			OutElevations[e] = ElStart + ElSpan * ((float)e / (float)(ElN - 1));
		}
	}
}

FVector UMjLidarSensor::SphericalDirectionMj(float AzimuthDeg, float ElevationDeg)
{
	const float Az = FMath::DegreesToRadians(AzimuthDeg);
	const float El = FMath::DegreesToRadians(ElevationDeg);
	const float Ce = FMath::Cos(El);
	return FVector(Ce * FMath::Cos(Az), Ce * FMath::Sin(Az), FMath::Sin(El));
}

void UMjLidarSensor::ClusterPoints(const TArray<FVector>& Points, float ThresholdCm,
	TArray<int32>& OutClusterIds, TArray<int32>& OutClusterSizes)
{
	const int32 N = Points.Num();
	OutClusterIds.Reset();
	OutClusterSizes.Reset();
	if (N == 0)
	{
		return;
	}

	if (ThresholdCm <= 0.0f)
	{
		// Degenerate threshold: every point is its own cluster.
		OutClusterIds.SetNum(N);
		OutClusterSizes.Init(1, N);
		for (int32 i = 0; i < N; ++i)
		{
			OutClusterIds[i] = i;
		}
		return;
	}

	// Union-find with path compression.
	TArray<int32> Parent;
	Parent.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Parent[i] = i;
	}
	auto Find = [&Parent](int32 X) {
		while (Parent[X] != X)
		{
			Parent[X] = Parent[Parent[X]];
			X = Parent[X];
		}
		return X;
	};
	auto Union = [&Parent, &Find](int32 A, int32 B) {
		const int32 Ra = Find(A);
		const int32 Rb = Find(B);
		if (Ra != Rb)
		{
			Parent[Rb] = Ra;
		}
	};

	// Spatial hash: cell edge = threshold, so any within-threshold pair must
	// land in the 3x3x3 neighbourhood.
	const float Cell = ThresholdCm;
	TMap<FIntVector, TArray<int32>> Grid;
	Grid.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		const FVector& P = Points[i];
		const FIntVector Key(
			FMath::FloorToInt(P.X / Cell),
			FMath::FloorToInt(P.Y / Cell),
			FMath::FloorToInt(P.Z / Cell));
		Grid.FindOrAdd(Key).Add(i);
	}

	const float ThrSq = ThresholdCm * ThresholdCm;
	for (int32 i = 0; i < N; ++i)
	{
		const FVector& P = Points[i];
		const FIntVector Key(
			FMath::FloorToInt(P.X / Cell),
			FMath::FloorToInt(P.Y / Cell),
			FMath::FloorToInt(P.Z / Cell));
		for (int32 Dx = -1; Dx <= 1; ++Dx)
		{
			for (int32 Dy = -1; Dy <= 1; ++Dy)
			{
				for (int32 Dz = -1; Dz <= 1; ++Dz)
				{
					const TArray<int32>* Bucket = Grid.Find(Key + FIntVector(Dx, Dy, Dz));
					if (!Bucket)
					{
						continue;
					}
					for (const int32 J : *Bucket)
					{
						if (J == i)
						{
							continue;
						}
						if (FVector::DistSquared(P, Points[J]) <= ThrSq)
						{
							Union(i, J);
						}
					}
				}
			}
		}
	}

	// Compact roots into cluster indices.
	OutClusterIds.SetNum(N);
	TMap<int32, int32> RootToCluster;
	RootToCluster.Reserve(N);
	int32 NumClusters = 0;
	for (int32 i = 0; i < N; ++i)
	{
		const int32 Root = Find(i);
		int32 Cluster = RootToCluster.FindRef(Root, -1);
		if (Cluster < 0)
		{
			Cluster = NumClusters++;
			RootToCluster.Add(Root, Cluster);
		}
		OutClusterIds[i] = Cluster;
	}

	OutClusterSizes.Init(0, NumClusters);
	for (int32 i = 0; i < N; ++i)
	{
		++OutClusterSizes[OutClusterIds[i]];
	}
}

float UMjLidarSensor::SampleGaussian(FRandomStream& Stream, float StdDev)
{
	if (StdDev <= 0.0f)
	{
		return 0.0f;
	}
	// Box-Muller. FRandomStream::FRand() is [0, 1).
	float U1 = Stream.FRand();
	const float U2 = Stream.FRand();
	U1 = FMath::Max(U1, 1e-6f);
	return StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);
}
