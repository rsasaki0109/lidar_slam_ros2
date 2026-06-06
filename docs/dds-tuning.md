# DDS / large-message tuning

LiDAR SLAM publishes large `sensor_msgs/PointCloud2` messages (a single
Ouster OS0-128 scan is **6 MB+**). Out of the box, ROS 2's default DDS settings
are not tuned for messages this large, which can cause **dropped scans** and
node-to-node delivery delay. This page explains the failure mode, the default
that avoids it, and the config to set when you do need online streaming.

## The failure mode

- A 6 MB+ PointCloud2 is fragmented across many UDP datagrams. With the default
  receive-buffer / fragment-reassembly limits, fragments are dropped under load
  and the subscriber never assembles a complete message.
- Symptoms: the front-end logs *"Subscribed …"* but never advances; scans are
  silently dropped; nodes that should coexist (e.g. a streaming odometry node
  alongside `graph_based_slam`) starve.
- This is a **transport** problem, not a SLAM problem — the same bag replays
  fine when read in-process.

## Default that avoids it: the offline node reads the bag in-process

The recommended public path (**RKO-LIO + graph_based_slam**) uses the
**offline node**, which reads the rosbag2 **internally** rather than
subscribing to a streamed topic. There is no large-PointCloud2 DDS hop, so the
drop/delay class above does not occur. This is why the benchmark and map
authoring scripts drive the offline node — it is the robust default, not a
limitation.

> If you only run the documented benchmark / map-authoring flows, you do **not**
> need any of the DDS tuning below.

## When you stream online: tune CycloneDDS + the kernel

If you must run a streaming pipeline (e.g. live sensor → online SLAM, or the
AWSIM × Autoware demo where AWSIM publishes the cloud over DDS), set both a
CycloneDDS profile **and** the kernel socket/fragment limits. The values below
are the ones validated in the AWSIM × Autoware demo
(`docs/awsim-autonomous-driving-tutorial.md`) and are a sane starting point for
loopback / single-host streaming.

### CycloneDDS profile

```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks><WhcHigh>500kB</WhcHigh></Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
EOF

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

- `SocketReceiveBufferSize` (10 MB) is the single most important knob for large
  PointCloud2 — it gives the kernel room to hold fragments until reassembly.
- The `lo` interface + `AllowMulticast` settings target single-host / loopback
  streaming. For a real multi-host robot network, set the interface to your NIC
  and size the buffers to the link.

### Kernel socket / fragment limits (requires `sudo`)

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo ip link set lo multicast on
```

- `net.core.rmem_max` must be ≥ the CycloneDDS `SocketReceiveBufferSize`, or the
  DDS request to enlarge the socket buffer is silently clamped.
- `ipfrag_*` raise the IP fragment-reassembly budget so 6 MB clouds reassemble
  under load instead of being dropped.

Persist these in `/etc/sysctl.d/` for a real deployment rather than setting them
per shell.

## Intra-process composition

Composing the SLAM nodes into one process with intra-process communication
sidesteps DDS for in-process hops entirely (zero-copy pointer passing). This is
the most robust answer for an online pipeline on a single host, but the public
launch files currently run nodes as separate processes. Intra-process
composition + FastDDS shared-memory / zero-copy transport is tracked as a future
hardening item; until it ships, prefer the offline node (default) or the
CycloneDDS + kernel tuning above for streaming.

## Quick reference

| Situation | What to do |
|---|---|
| Benchmark / map authoring (documented flows) | Nothing — the offline node reads the bag in-process |
| Live sensor → online SLAM, single host | CycloneDDS profile + kernel sysctl above |
| Multi-host robot network | Same, but size `SocketReceiveBufferSize` / `rmem_max` to the link and bind the real NIC |
| Dropped scans despite tuning | Verify `rmem_max` ≥ CycloneDDS buffer; confirm `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI` are exported in the node's environment |
