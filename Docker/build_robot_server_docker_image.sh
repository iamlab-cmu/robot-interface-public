cd robot-server

mkdir -p .docker_util_scripts
cp ../docker_util_scripts/*.sh .docker_util_scripts/

docker build \
    --squash \
    -t iamlab/robot-server:latest \
    -t iamlab/robot-server:$1 \
    --build-arg SSH_KEY="$(cat ssh/iamlab_docker_robot-interface_id_rsa_server)" \
    .