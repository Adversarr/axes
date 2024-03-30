#!/usr/bin/env bash
if [ -z "$1" ]; then
    echo "Usage: $0 <path-to-sdk>"
    exit 1
fi

export SDK_PATH="$(realpath $1)"
echo "SDK path: $SDK_PATH"
if [ ! -d "$SDK_PATH" ]; then
    echo "WARN: SDK path does not exist"
    exit 1
fi

export BUILD_TYPE=$2
if [ -z "$2" ]; then
  export BUILD_TYPE=RelWithDebInfo
  echo "Warning: BUILD_TYPE not set, using default: $BUILD_TYPE"
fi

cd "$SDK_PATH/.."
echo "==> Cloning axdeps repository"
if [ -d "axdeps" ]; then
    echo "Info: axdeps directory already exists"
else
    git clone https://github.com/Adversarr/axdeps.git
    if [ $? -ne 0 ]; then
        echo "Error: axdeps clone failed"
        exit 1
    fi
fi

echo "==> Building axdeps"
SDK_PATH=$SDK_PATH BUILD_TYPE=$BUILD_TYPE ./axdeps/build.sh
RET=$?
if [ $RET -ne 0 ]; then
    echo "Error: axdeps build failed"
    exit 1
fi
echo "==> Build and Install axdeps completed."
echo "==> Removing axdeps directory"
read -p "Do you want to remove axdeps directory? (y/n): (default: n) " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -rf axdeps/build
    echo "Info: axdeps directory removed"
fi

echo "Success: axdeps build completed, Remember to set AX_SDK_PATH to $SDK_PATH when running cmake."
