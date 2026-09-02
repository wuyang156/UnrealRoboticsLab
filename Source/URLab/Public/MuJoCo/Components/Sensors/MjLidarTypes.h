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
#include "MjLidarTypes.generated.h"

/**
 * @enum EMjLidarOutputMode
 * @brief Which outputs a UMjLidarSensor computes and publishes each scan.
 *
 * - Points: publishes the per-beam hit points only (cheapest).
 * - Targets: clusters hits into aggregated targets only; LastPoints stays empty.
 * - Both: points and targets.
 */
UENUM(BlueprintType)
enum class EMjLidarOutputMode : uint8
{
	Points UMETA(DisplayName = "Points"),
	Targets UMETA(DisplayName = "Targets"),
	Both UMETA(DisplayName = "Points and Targets")
};

/**
 * @struct FMjLidarPoint
 * @brief One lidar beam return.
 *
 * WorldPos is in Unreal world coordinates (centimetres), converted from the
 * MuJoCo-space hit computed on the physics thread. Azimuth/Elevation are the
 * beam angles actually used for the cast (post-noise, if noise is enabled),
 * in degrees, following the sensor convention documented on UMjLidarSensor:
 * azimuth sweeps around the sensor's local +Z, elevation is positive above
 * the sensor's local X-XY plane, and 0/0 points along local +X.
 */
USTRUCT(BlueprintType)
struct FMjLidarPoint
{
	GENERATED_BODY()

	/** Hit position in Unreal world coordinates, centimetres. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	FVector WorldPos = FVector::ZeroVector;

	/** Beam azimuth in degrees (post-noise). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	float AzimuthDeg = 0.0f;

	/** Beam elevation in degrees (post-noise). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	float ElevationDeg = 0.0f;

	/** Range in metres; negative when the beam missed (or fell inside the blind zone). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	float RangeM = -1.0f;

	/** MuJoCo geom id of the hit surface, or -1 on a miss. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	int32 HitGeomId = -1;

	/** Id of the scan this point belongs to (monotonic per sensor). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	int64 FrameId = -1;
};

/**
 * @struct FMjLidarTarget
 * @brief One aggregated target: a cluster of contemporaneous hit points.
 *
 * RadialSpeed is a kinematic approximation (centroid differencing between
 * scans), NOT a Doppler measurement: it is the component of the centroid's
 * world velocity along the line of sight from the sensor, positive when the
 * target recedes.
 */
USTRUCT(BlueprintType)
struct FMjLidarTarget
{
	GENERATED_BODY()

	/** Persistent id of the tracked cluster (matches across consecutive scans while matched). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	int32 TargetId = -1;

	/** Centroid position relative to the sensor, Unreal coordinates, centimetres. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	FVector RelativePos = FVector::ZeroVector;

	/** Kinematic radial speed in m/s: positive when receding from the sensor. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	float RadialSpeed = 0.0f;

	/** Number of hit points in the cluster. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	int32 NumPoints = 0;

	/** Cluster centroid in Unreal world coordinates, centimetres. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	FVector CentroidWorldPos = FVector::ZeroVector;
};

/**
 * @struct FMjLidarScan
 * @brief A complete scan result: points and/or targets plus scan metadata.
 */
USTRUCT(BlueprintType)
struct FMjLidarScan
{
	GENERATED_BODY()

	/** Hit points (empty in Targets output mode). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	TArray<FMjLidarPoint> Points;

	/** Aggregated targets (empty in Points output mode). */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	TArray<FMjLidarTarget> Targets;

	/** Monotonic scan id. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	int64 ScanId = -1;

	/** MuJoCo simulation time (d->time) at which the scan was taken, seconds. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Lidar")
	double SimTime = 0.0;
};

/** Broadcast on the game thread after each completed scan is consumed. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnLidarScan, FMjLidarScan, Scan);
