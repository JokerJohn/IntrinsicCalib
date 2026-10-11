# Sourced by the run scripts: give every run its own rosmaster on a free port so that concurrent runs
# (or a pre-existing roscore) never share node names or the /use_sim_time parameter.
pick_free_port() {
  local p
  for p in $(shuf -i 12000-12999 -n 50); do
    if ! (ss -ltn 2>/dev/null | awk '{print $4}' | grep -q ":$p\$"); then echo $p; return 0; fi
  done
  echo 11311
}
export ROS_PORT=$(pick_free_port)
export ROS_MASTER_URI=http://localhost:$ROS_PORT
export ROS_HOSTNAME=localhost
