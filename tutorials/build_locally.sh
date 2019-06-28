#!/bin/sh

if [ -d "./build" ]; then
  echo "Doc already built, build again."
else
  echo "Doc not built yet."
  # Install dependencies
  sudo apt install ros-melodic-rosdoc-lite

  # Setup Environment
  rm -rf build
fi

# Build
rosdoc_lite -o build . | grep 'WARNING\|ERROR'

# Run
xdg-open ./build/html/index.html 
