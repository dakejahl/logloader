## Load ur logs
Downloads PX4 logs from the SD card and uploads to [PX4 Flight Review](https://review.px4.io/).

The **config.toml** file is used to configure the program settings.
https://github.com/ARK-Electronics/logloader/blob/a316ebd0380bd44f5b57496ab583768702867f4f/config.toml#L1-L7

### Behavior
Downloading and uploading operations are performed in separate threads. The downloading thread will only download logs with a datetime greater than the most recent log found locally in the `logging_directory`. If no logs are found locally only the most recent log will be downloaded. The upload thread will only upload logs that are not recorded in the `uploaded_logs_file`. Logs are named with the ISO 8601 date and time format **yyyy-mm-ddThh:mm:ssZ.ulg**.

### Build
```
make
```

### Run
```bash
./build/logloader | tee output.txt
```

### Performance
Monitor network traffic
```
sudo iftop -i wlo1
```
Or use a Wireshark filter
```
mavlink_proto.msgid == 117 || mavlink_proto.msgid == 118 || mavlink_proto.msgid == 119 || mavlink_proto.msgid == 120
```
