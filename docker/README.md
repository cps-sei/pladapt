# Building and Running in Docker
**NOTE:** this is work in progress and not fully documented

To provide a portable and consistent build environment, PLADAPT can be built in a Docker container that provides all the necessary dependencies. With this approach, all the files are in the host file sytem and all the build tools and libraries are in the container. In this way, it is possible to use an IDE like Eclipse to work on a local copy of the source code, and have the build execute in the container.

To do this, it is necessary to build the docker image first. This is done by executing:

```
$PLADAPT/docker/build.sh
```
 
This command generates a Docker image named `pladevbase`, which is used to both build and execute PLADAPT.
 
## Working with Eclipse CDT
- Install Eclipse CDT 9.8.1 or later.
- In Eclipse CDT:
    - Use File|Import...|General|Existing Project into Workspace to import PLADAPT Eclipse project into workspace
    - In Project Properties|C/C++ Build|Settings, check Build inside Docker Image and Run all Autotools in Container. Select the `pladevbase` image.
- Right-click project|Reconfigure Project
- Right-click project|Build Project
- Right-click project|Index|Rebuild



