#!/bin/bash

# Variables
REMOTE_USER="reuben"
REMOTE_HOST="172.27.203.153"
REMOTE_IMAGE_PATH="dev/capstone/jetson/display/state.jpg"
REMOTE_FILE="dev/capstone/jetson/display/state.jpg"
PASS_NAME="reuben_at_jnano"


LOCAL_DESTINATION="display"


# Retrieve password using pass
PASSWORD=$(pass show $PASS_NAME)

# Function to sync the remote file to local
sync_remote_to_local() {
    sshpass -p "$PASSWORD" rsync -avz -e "ssh -p 22" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_IMAGE_PATH}" "$LOCAL_DESTINATION"
}

# while true; do

#     # Store the start time
#     # start_time=$(date +%s)

#     # Copy image file from remote device
#     sync_remote_to_local

#     # Store the end time
#     # end_time=$(date +%s)

#     # Calculate the time difference
#     # time_diff=$((end_time - start_time))

#     # Print the time difference
#     # echo "Time to download image: $time_diff seconds"
# done

while true; do
    sync_remote_to_local
done

# Monitor for changes to the remote file
# inotifywait -m -e close_write "${LOCAL_DESTINATION}" |
# while read -r directory events filename; do
#   echo $filename
#     if [[ "$filename" == "$(basename "$REMOTE_FILE")" ]]; then
#         echo "Remote file changed. Syncing to local..."
#         sync_remote_to_local
#     fi
# done
