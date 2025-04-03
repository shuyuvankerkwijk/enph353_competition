#!/bin/bash

# Create directories if they don't exist
mkdir -p OffRoad ramp Gravel Road

# Move files to their respective folders
for file in image_*_OffRoad_*.png; do
	[ -e "$file" ] || continue  # Handle case with no matches
	mv "$file" OffRoad/images/
done

for file in image_*_ramp_*.png; do
	[ -e "$file" ] || continue
	mv "$file" ramp/images/
done

for file in image_*_Gravel_*.png; do
	[ -e "$file" ] || continue
	mv "$file" Gravel/images/
done

for file in image_*_Road_*.png; do
	[ -e "$file" ] || continue
	mv "$file" Road/images/
done
