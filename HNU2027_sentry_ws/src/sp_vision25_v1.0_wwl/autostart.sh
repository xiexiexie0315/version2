sleep 5
cd ~/version2/pb2025_sp2025_sentry_ws/pb2025_sp2025_sentry_ws/src/sp_vision25_v1.0_wwl
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
