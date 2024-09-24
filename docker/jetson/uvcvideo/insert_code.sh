#!/bin/bash

# Target file
input_file="uvc_video.c"  # Change to the name of your target file

# Temporary file
temp_file=$(mktemp)

# Flag variable
found_func=false
insert_code=false

# Process the file line by line
while IFS= read -r line; do
    # If "uvc_fixup_video_ctrl" is found, set the flag
    if [[ "$line" == *"uvc_fixup_video_ctrl"* ]]; then
        found_func=true
    fi

    # If "}" is found and "uvc_fixup_video_ctrl" has been encountered
    if [[ "$found_func" == true && "$line" == "}" && "$insert_code" == false ]]; then
        echo "        if (format->flags & UVC_FMT_FLAG_COMPRESSED) {" >> "$temp_file"
        echo "                ctrl->dwMaxPayloadTransferSize = 0x400;" >> "$temp_file"
        echo "        }" >> "$temp_file"
        insert_code=true
    fi

    # Output the current line
    echo "$line" >> "$temp_file"

    # Reset the flag if "}" is found
    if [[ "$line" == "}" ]]; then
        found_func=false
    fi
done < "$input_file"

# Replace the original file with the temporary file
mv "$temp_file" "$input_file"
