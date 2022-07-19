```json
{
    "monitorName" : "rgbd_monitor",
    "monitorDescription" : "Monitor verifying that the pointcloud of the RGBD camera has no NaNs",
    "healthStatus":
    {
        "nans": true
    }
}
```

```json
{
  "source_id":"<unique>",
  "target_id":["<monitorName>"],
  "message":{
    "command":"shutdown",
    "status" :"success/failure/fatal",
    "thread_id":"<source_thread_id>"
  },
  "type":"ack/cmd/helo"
}
```
