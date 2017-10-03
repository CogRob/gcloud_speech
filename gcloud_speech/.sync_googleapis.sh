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

mkdir -p googleapis

GOOGLEAPIS_STATUS_OUTPUT=$BASH_SCRIPT_PATH/googleapis/.GIT_STATUS
date >> $GOOGLEAPIS_STATUS_OUTPUT
cd googleapis_download
echo "googleapis/googleapis branch: "$(git branch | grep \* | cut -d ' ' -f2-) \
    >> $GOOGLEAPIS_STATUS_OUTPUT
echo $(git rev-parse HEAD)  >> $GOOGLEAPIS_STATUS_OUTPUT
if GIT_CLEAN=$(git status --porcelain) && [ -z "$GIT_CLEAN" ]; then
  echo "Working directory clean." >> $GOOGLEAPIS_STATUS_OUTPUT
else
  echo "Warning: Uncommitted changes in source workspace."
  echo "Uncommitted changes." >> $GOOGLEAPIS_STATUS_OUTPUT
fi

cd $BASH_SCRIPT_PATH

rm -rf googleapis_download/.git

rsync \
  --prune-empty-dirs --archive \
  --include="*/" --include="*.proto" --exclude="*" \
  googleapis_download/ \
  googleapis/

find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)#\1gcloud_speech/googleapis/#g' {} \;
find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)gcloud_speech/googleapis/\(google/protobuf\)#\1\2#g' {} \;
find ./googleapis -type f -name \*.proto -exec sed -i -e '1s#^#// This file is modified by CogRob to work with Catkin/ROS.\n\n#' {} \;

# The grpc library compiled and linked status.proto with a different path.
# We need to match this path to avoid descriptor conflicts.
find ./googleapis -type f -exec sed -i -e 's#\(import\s\+\"\)gcloud_speech/googleapis/google/rpc/status.proto#\1src/proto/grpc/status/status.proto#g' {} \;
rm googleapis/google/rpc/status.proto

cp googleapis_download/LICENSE googleapis/
cp googleapis_download/README.md googleapis/

sed -i '1i# These files are modified by CogRob to work with Catkin/ROS.' googleapis/README.md

cd $BASH_SCRIPT_PATH
rm -rf googleapis_download
