#!/usr/bin/env bash

# Get script path.
BASH_SCRIPT="${BASH_SOURCE[0]}"
while [ -h "$BASH_SCRIPT" ]; do
  BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"
  BASH_SCRIPT="$(readlink "$BASH_SCRIPT")"
  [[ $BASH_SCRIPT != /* ]] && BASH_SCRIPT="$BASH_SCRIPT_PATH/$BASH_SCRIPT"
done
BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"

cd $BASH_SCRIPT_PATH
rm -rf googleapis googleapis_download

git clone https://github.com/googleapis/googleapis.git googleapis_download
rm -rf googleapis_download/.git

rsync \
  --prune-empty-dirs --archive \
  --include="*/" --include="*.proto" --exclude="*" \
  googleapis_download/ \
  googleapis/

find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)#\1gcloud_speech/googleapis/#g' {} \;
find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)gcloud_speech/googleapis/\(google/protobuf\)#\1\2#g' {} \;
find ./googleapis -type f -exec sed -i -e '1s#^#// This file is modified by CogRob to work with Catkin/ROS.\n\n#' {} \;

# The grpc library compiled and linked status.proto with a different path.
# We need to match this path to avoid descriptor conflicts.
find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)gcloud_speech/googleapis/google/rpc/status.proto#\1src/proto/grpc/status/status.proto#g' {} \;
rm googleapis/google/rpc/status.proto

cp googleapis_download/LICENSE googleapis/
cp googleapis_download/README.md googleapis/

rm -rf googleapis_download
