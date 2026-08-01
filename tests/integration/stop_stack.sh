#!/usr/bin/env bash
# Stop everything the stack starts. Kill by PID: `pkill -f <pattern>` matches the
# calling shell's own command line and kills it (seen as exit code 144).
ps aux | grep -E "[a]sync_slam_toolbox|[p]arameter_bridge|[s]tatic_transform_pub|[l]ifecycle_manager|[g]z sim|[a]dapter_sim|[p]ointcloud_to_laserscan" \
  | awk '{print $2}' | while read -r p; do kill -9 "$p" 2>/dev/null; done
sleep 2
echo "gz:$(ps aux|grep -c '[g]z sim') slam:$(ps aux|grep -c '[a]sync_slam') bridge:$(ps aux|grep -c '[p]arameter_bridge')"
