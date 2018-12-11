# Use this command to create the docker image to build slam3d with plain CMake
docker build --no-cache -t d-reg.hb.dfki.de/slam3d/ci-plain:latest .

# Push it to DFKI's local docker-registry
docker login d-reg.hb.dfki.de (use domain account)
docker push d-reg.hb.dfki.de/slam3d/ci-plain:latest

# Do a test-build using the newly created container
docker run -it -v PATH_TO_SLAM3D_SOURCE:/source/slam3d d-reg.hb.dfki.de/slam3d/ci-plain