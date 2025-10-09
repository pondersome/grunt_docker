# GitHub Container Registry (GHCR) Setup Guide

Complete walkthrough for setting up GitHub Container Registry for the grunt_docker multi-architecture images.

---

## Why GHCR?

**Benefits for grunt_docker:**
- ✅ No new account needed (use existing GitHub account)
- ✅ Packages automatically linked to your GitHub repos
- ✅ Higher pull rate limits than Docker Hub (authenticated users)
- ✅ Seamless GitHub Actions CI/CD integration (future)
- ✅ Free for unlimited public images
- ✅ Private images available (500MB free, then paid tiers)

**Trade-offs:**
- ❌ Slightly longer image names: `ghcr.io/pondersome/` vs `pondersome/`
- ❌ Images are private by default (must manually make public)
- ❌ Less "standard" than Docker Hub (some tutorials assume Docker Hub)

---

## Prerequisites

- GitHub account (you have: `pondersome`)
- Docker or Docker Desktop installed
- Command-line access (WSL2, Linux, or macOS terminal)

---

## Step 1: Create Personal Access Token (PAT)

### 1.1 Navigate to GitHub Token Settings

**Option A: Direct Link**
- Go to: https://github.com/settings/tokens

**Option B: Through GitHub UI**
1. Click your profile icon (top-right)
2. Settings → Developer settings (bottom of left sidebar)
3. Personal access tokens → Tokens (classic)

### 1.2 Generate New Token

1. Click **"Generate new token (classic)"**
   - _Note: "Fine-grained tokens" are newer but have limitations with packages_

2. Fill out the form:
   - **Note**: `Docker BuildX for grunt_docker`
   - **Expiration**: Choose based on your preference
     - `90 days` (recommended for security)
     - `No expiration` (convenient but less secure)

3. **Select scopes** (permissions):
   ```
   ✅ write:packages    (Upload packages to GHCR)
   ✅ read:packages     (Download packages from GHCR)
   ✅ delete:packages   (Optional: delete old package versions)
   ☐ repo              (NOT needed for public images)
   ```

4. Click **"Generate token"** at bottom

### 1.3 Copy Token

**IMPORTANT:** Copy the token immediately — you can't see it again!

```
Token will look like: ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

**Security Note:** Treat this like a password. Anyone with this token can push/pull packages as you.

---

## Step 2: Save Token Securely on Your Workstation

### On WSL2 / Linux

```bash
# 1. Export to environment (for this session)
export GITHUB_PAT="ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# 2. Save to a secure file
echo $GITHUB_PAT > ~/.github-docker-token
chmod 600 ~/.github-docker-token

# 3. Add to .bashrc for automatic loading in future sessions
echo '# Load GitHub PAT for Docker pushes' >> ~/.bashrc
echo 'export GITHUB_PAT=$(cat ~/.github-docker-token 2>/dev/null)' >> ~/.bashrc

# 4. Reload .bashrc
source ~/.bashrc

# 5. Verify it's set
echo $GITHUB_PAT
# Should print: ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

### On macOS (same as Linux above)

```bash
# Same commands as Linux/WSL2
export GITHUB_PAT="ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
echo $GITHUB_PAT > ~/.github-docker-token
chmod 600 ~/.github-docker-token
echo 'export GITHUB_PAT=$(cat ~/.github-docker-token 2>/dev/null)' >> ~/.zshrc  # or ~/.bashrc
source ~/.zshrc  # or ~/.bashrc
```

---

## Step 3: Login to GHCR

### 3.1 Using Environment Variable (Recommended)

```bash
echo $GITHUB_PAT | docker login ghcr.io -u pondersome --password-stdin
```

**Expected output:**
```
WARNING! Your password will be stored unencrypted in /home/karim/.docker/config.json.
Configure a credential helper to remove this warning. See
https://docs.docker.com/engine/reference/commandline/login/#credentials-store

Login Succeeded
```

### 3.2 Alternative: Interactive Login

If the above doesn't work, try interactive:

```bash
docker login ghcr.io -u pondersome
# Password: [paste your ghp_xxx token here]
```

### 3.3 Verify Login

```bash
# Check that ghcr.io is in your auth list
cat ~/.docker/config.json | grep ghcr.io
```

Should show something like:
```json
"ghcr.io": {}
```

---

## Step 4: Test with Your First Image Push

### 4.1 Build a Test Image (Local Only)

```bash
cd /home/karim/grunt_docker

# Build for your current architecture only
docker buildx build \
  --platform linux/amd64 \
  --load \
  -t ghcr.io/pondersome/grunt_base:test \
  -f base/Dockerfile \
  .
```

**This will take 10-20 minutes** (downloads base images, installs ROS, builds workspace).

### 4.2 Test the Image Locally

```bash
docker run -it --rm ghcr.io/pondersome/grunt_base:test bash

# Inside container, test ROS 2 is working:
ros2 topic list
# Should show: /parameter_events, /rosout

# Exit container
exit
```

### 4.3 Push to GHCR

```bash
docker push ghcr.io/pondersome/grunt_base:test
```

**Expected output:**
```
The push refers to repository [ghcr.io/pondersome/grunt_base]
a1b2c3d4e5f6: Pushed
...
test: digest: sha256:abc123... size: 8710 # Size will vary
```

**If you see an error:**
```
unauthorized: unauthenticated: User cannot be authenticated with the token provided.
```

→ Go back to Step 3.1 and re-login.

---

## Step 5: Make Package Public

By default, GHCR packages are **private**. To make them public (required for free unlimited storage):

### 5.1 Navigate to Your Packages

**Option A: Direct Link**
- Go to: https://github.com/pondersome?tab=packages

**Option B: From GitHub Profile**
1. Visit your profile: https://github.com/pondersome
2. Click **"Packages"** tab

### 5.2 Find Your Package

You should see: `grunt_base` (or whatever you named your image)

Click on it.

### 5.3 Change Visibility

1. Click **"Package settings"** (right sidebar)
2. Scroll to **"Danger Zone"**
3. Click **"Change visibility"**
4. Select **"Public"**
5. Type the package name to confirm
6. Click **"I understand, change package visibility"**

### 5.4 Link Package to Repository (Optional but Recommended)

Still in Package settings:

1. Scroll to **"Connect repository"**
2. Select `pondersome/grunt_docker`
3. Click **"Connect repository"**

Now the package will show up on your repo's main page!

---

## Step 6: Test Pull from Another Machine

### On the Same Machine (Simulating Pull)

```bash
# Remove local image
docker rmi ghcr.io/pondersome/grunt_base:test

# Pull from GHCR (no login required for public images!)
docker pull ghcr.io/pondersome/grunt_base:test

# Run it
docker run -it --rm ghcr.io/pondersome/grunt_base:test bash
```

### On Another Machine (e.g., Betty Jetson)

```bash
# No login needed for public images!
docker pull ghcr.io/pondersome/grunt_base:test
docker run -it --rm ghcr.io/pondersome/grunt_base:test bash
```

---

## Multi-Architecture Push (Final Test)

Now that you've verified the process, do a real multi-arch build:

```bash
cd /home/karim/grunt_docker

# Create buildx builder (if not done already)
docker buildx create --name gruntbuilder --use

# Build and push for both x86_64 and ARM64
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --push \
  -t ghcr.io/pondersome/grunt_base:jazzy \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg GZ_VERSION=gz-harmonic \
  -f base/Dockerfile \
  .
```

**This will take 30-60 minutes** (builds twice: once for amd64, once for arm64).

### Verify Multi-Arch Manifest

```bash
# Inspect the manifest list
docker buildx imagetools inspect ghcr.io/pondersome/grunt_base:jazzy
```

Expected output:
```
Name:      ghcr.io/pondersome/grunt_base:jazzy
MediaType: application/vnd.docker.distribution.manifest.list.v2+json
Digest:    sha256:abc123...

Manifests:
  Name:      ghcr.io/pondersome/grunt_base:jazzy@sha256:def456...
  MediaType: application/vnd.docker.distribution.manifest.v2+json
  Platform:  linux/amd64

  Name:      ghcr.io/pondersome/grunt_base:jazzy@sha256:ghi789...
  MediaType: application/vnd.docker.distribution.manifest.v2+json
  Platform:  linux/arm64
```

---

## Troubleshooting

### Problem: "unauthorized: unauthenticated"

**Solution:**
```bash
# Re-login with explicit token
echo $GITHUB_PAT | docker login ghcr.io -u pondersome --password-stdin

# If that fails, check token is set:
echo $GITHUB_PAT
# Should print ghp_xxx..., not blank

# If blank, reload .bashrc:
source ~/.bashrc
```

### Problem: "denied: permission_denied"

**Causes:**
- Token doesn't have `write:packages` permission
- Token has expired

**Solution:**
1. Go to https://github.com/settings/tokens
2. Find your token
3. Regenerate with correct scopes
4. Update `~/.github-docker-token` with new value
5. Re-login

### Problem: "Invalid ELF image for this architecture" (Multi-Arch Builds)

**Error message:**
```
.buildkit_qemu_emulator: /bin/sh: Invalid ELF image for this architecture
ERROR: failed to solve: process "/dev/.buildkit_qemu_emulator..." did not complete successfully
```

**Cause:** QEMU emulators not installed or builder created before QEMU was available.

**Solution:**
```bash
# 1. Install QEMU emulators for all architectures
docker run --privileged --rm tonistiigi/binfmt --install all

# 2. Remove old builder and create new one
docker buildx rm gruntbuilder
docker buildx create --name gruntbuilder --driver docker-container --bootstrap --use

# 3. Verify ARM64 support is present
docker buildx inspect gruntbuilder | grep -i platforms
# Should show: linux/amd64, ..., linux/arm64, ...

# 4. Retry your multi-arch build
docker buildx build --platform linux/amd64,linux/arm64 --push ...
```

### Problem: "Error response from daemon: no matching manifest"

**Cause:** Trying to pull multi-arch image on platform not included in build.

**Solution:**
```bash
# Check what platforms were built:
docker buildx imagetools inspect ghcr.io/pondersome/grunt_base:jazzy

# Rebuild with correct --platform flag:
docker buildx build --platform linux/amd64,linux/arm64 ...
```

### Problem: Push is slow / times out

**Causes:**
- Large image size (ROS + Gazebo = 3-5GB per arch)
- Slow internet connection
- Docker Desktop resource limits

**Solutions:**
1. **Increase Docker Desktop resources:**
   - Settings → Resources → increase disk image size to 100GB+
   - Increase CPU/Memory allocation

2. **Push during off-peak hours**

3. **Use wired ethernet** instead of WiFi

4. **Clean up old images** to free space:
   ```bash
   docker system prune -a
   docker buildx prune -a
   ```

### Problem: "failed to solve: Canceled"

**Cause:** Build ran out of disk space or Docker resources.

**Solution:**
```bash
# Check disk usage:
df -h
docker system df

# Clean up:
docker system prune -a --volumes
docker buildx prune -a

# Increase Docker Desktop disk limit (Settings → Resources)
```

---

## Best Practices

### 1. Token Management

**DO:**
- ✅ Use 90-day expiration and set calendar reminder to renew
- ✅ Store token in `~/.github-docker-token` with `chmod 600`
- ✅ Generate separate tokens for different machines

**DON'T:**
- ❌ Commit token to git
- ❌ Share token with others
- ❌ Use same token for multiple purposes (e.g., CI/CD + personal)

### 2. Image Tagging Strategy

**For development:**
```bash
# Use descriptive test tags
-t ghcr.io/pondersome/grunt_base:test
-t ghcr.io/pondersome/grunt_base:jazzy-test
-t ghcr.io/pondersome/grunt_base:jazzy-20250108
```

**For releases:**
```bash
# Semantic versioning
-t ghcr.io/pondersome/grunt_base:v0.1.0
-t ghcr.io/pondersome/grunt_base:v0.1
-t ghcr.io/pondersome/grunt_base:v0
-t ghcr.io/pondersome/grunt_base:latest
```

### 3. Cleaning Up Old Versions

GHCR keeps all pushed versions. Delete old ones to save storage:

1. Go to: https://github.com/pondersome?tab=packages
2. Click package → Package settings
3. Scroll to **"Manage versions"**
4. Check versions to delete
5. Click **"Delete"**

### 4. Rate Limits

GHCR rate limits (per hour):
- **Authenticated pulls**: 15,000 requests
- **Anonymous pulls**: 500 requests (from same IP)

For grunt_docker (public images), anonymous pulls are fine. But if you hit limits:
```bash
# Pull with auth (uses your higher limit)
echo $GITHUB_PAT | docker login ghcr.io -u pondersome --password-stdin
docker pull ghcr.io/pondersome/grunt_base:jazzy
```

---

## Next Steps

### For grunt_docker Project

1. ✅ GHCR setup complete
2. ⬜ Build and push base image (Jazzy + Humble)
3. ⬜ Update compose files to use `ghcr.io/pondersome/*` images
4. ⬜ Test multi-arch pulls on:
   - Windows/WSL2 workstation (x86_64)
   - Betty Jetson (arm64)
   - BamBam RPi5 (arm64)
5. ⬜ Document image tags in README.md
6. ⬜ Set up GitHub Actions to auto-build on pushes (future)

### Useful Commands Reference

```bash
# Login
echo $GITHUB_PAT | docker login ghcr.io -u pondersome --password-stdin

# Build + push single arch (local testing)
docker buildx build --platform linux/amd64 --load -t ghcr.io/pondersome/grunt_base:test -f base/Dockerfile .
docker push ghcr.io/pondersome/grunt_base:test

# Build + push multi-arch (production)
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --push \
  -t ghcr.io/pondersome/grunt_base:jazzy \
  -f base/Dockerfile .

# Inspect multi-arch manifest
docker buildx imagetools inspect ghcr.io/pondersome/grunt_base:jazzy

# Pull (no auth needed for public)
docker pull ghcr.io/pondersome/grunt_base:jazzy

# Logout
docker logout ghcr.io
```

---

## Additional Resources

- **Official GHCR docs**: https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry
- **Docker buildx docs**: https://docs.docker.com/build/building/multi-platform/
- **Multi-arch best practices**: https://www.docker.com/blog/multi-arch-images/
- **GitHub PAT docs**: https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token

---

## Summary

You've successfully set up GHCR for the grunt_docker project!

**What you can now do:**
- ✅ Build images locally
- ✅ Push multi-architecture images to GHCR
- ✅ Pull images on any machine (x86_64 or ARM64)
- ✅ Share images publicly or keep them private
- ✅ Integrate with GitHub repos and future CI/CD

**Key takeaway:** GHCR + buildx enables you to build **once** on your x86_64 laptop and deploy to **both** x86_64 workstations and ARM64 robots (Betty, BamBam) without separate build steps.
