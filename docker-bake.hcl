// Buildx bake file - encodes the image matrix for grunt_docker
//
// Local (native arch, loads into docker):
//   docker buildx bake                      # base + dev for jazzy
//   docker buildx bake dev                  # just the dev image
//
// Publish multi-arch to GHCR (requires docker login ghcr.io):
//   docker buildx bake release --push
//   DATE_TAG=$(date +%Y%m%d) docker buildx bake release --push   # adds immutable date tags
//
// Legacy humble build (frozen; only rebuild deliberately):
//   ROS_DISTRO=humble docker buildx bake release --push

variable "ROS_DISTRO" {
  default = "jazzy"
}

variable "REGISTRY" {
  default = "ghcr.io/pondersome"
}

// Set to a YYYYMMDD string to also push immutable date tags
variable "DATE_TAG" {
  default = ""
}

group "default" {
  targets = ["base", "dev"]
}

group "release" {
  targets = ["base-release", "dev-release"]
}

target "_common" {
  dockerfile = "base/Dockerfile"
  context    = "."
  args = {
    ROS_DISTRO = ROS_DISTRO
    GZ_VERSION = "gz-harmonic"
  }
}

// --- Local single-arch builds (fast, load into docker) ---

target "base" {
  inherits = ["_common"]
  target   = "base"
  tags     = ["${REGISTRY}/grunt:${ROS_DISTRO}"]
}

target "dev" {
  inherits = ["_common"]
  target   = "dev"
  tags     = ["${REGISTRY}/grunt:${ROS_DISTRO}-dev"]
}

// --- Multi-arch release builds (push to GHCR) ---

target "base-release" {
  inherits  = ["base"]
  platforms = ["linux/amd64", "linux/arm64"]
  tags = compact([
    "${REGISTRY}/grunt:${ROS_DISTRO}",
    DATE_TAG != "" ? "${REGISTRY}/grunt:${ROS_DISTRO}-${DATE_TAG}" : "",
  ])
}

target "dev-release" {
  inherits  = ["dev"]
  platforms = ["linux/amd64", "linux/arm64"]
  tags = compact([
    "${REGISTRY}/grunt:${ROS_DISTRO}-dev",
    DATE_TAG != "" ? "${REGISTRY}/grunt:${ROS_DISTRO}-dev-${DATE_TAG}" : "",
  ])
}
