cd robot-client

mkdir -p .docker_util_scripts
cp ../docker_util_scripts/*.sh .docker_util_scripts/

docker build \
    --squash \
    -t iamlab/robot-client:latest \
    -t iamlab/robot-client:$1 \
    --build-arg SSH_KEY="$(cat ssh/iamlab_docker_robot-interface_id_rsa_client)" \
    --build-arg ROBOT_SERVER_IP="192.168.1.3" \
    .