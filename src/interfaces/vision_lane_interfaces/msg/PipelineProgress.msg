# Periodic progress of one pipeline stage, published one-way on /pipeline/progress.
# Consumers (the map fusion node, evaluation tools) decide; producers never wait.
std_msgs/Header header             # stamp: the node's clock when sent
string node                        # stage name, e.g. lane_map, occupancy, pose, lane_detection
string output_topic                # main output this progress describes
# Watermark: every input with capture time <= watermark has been fully handled
# (published, merged or dropped); nothing older will appear on output_topic.
# It never moves backwards. Capture time, not processing time.
builtin_interfaces/Time watermark
bool watermark_valid               # false until the first input is handled
float64 latency_ms                 # header.stamp - watermark; -1 when not valid
uint32 queue_size                  # inputs held inside the stage, not yet final
uint64 processed_total             # inputs that reached the output
uint64 dropped_total               # inputs dropped for good
# Times the stage's own candidate fell below the already-published watermark
# (an older input showed up after its time was declared final). The published
# watermark stays put; anything above 0 means finality was claimed too early.
uint64 watermark_regressions
