#!/bin/bash -e
# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)

set -x

timestamp=$(date +%Y_%m_%d-%H_%M_%S)
cd ${SOURCE_DIRECTORY}
mkdir $timestamp
mkdir $timestamp/overlays

bcm_types='bcm2709 bcm2711 bcmrpi'

for bcm in $bcm_types; do
	mv adi_${bcm}_defconfig/zImage ./$timestamp/zImage
	mv adi_${bcm}_defconfig/*.dtbo ./$timestamp/overlays
done

curl -u$USERNAME:$PASSWORD -T ${SOURCE_DIRECTORY}/${timestamp} $ARTIFACTORY_PATH
