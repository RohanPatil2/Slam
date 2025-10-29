# How to Push This Project to GitHub

## Step-by-Step Instructions

### Step 1: Create a New Repository on GitHub

1. Go to https://github.com
2. Click the **"+"** icon (top right) → **"New repository"**
3. Repository name: `uncertainty-aware-slam` (or your choice)
4. Description: `Real-Time Uncertainty-Aware 2D SLAM with Entropy Quantification`
5. **Keep it Public** or Private (your choice)
6. **DO NOT** check "Initialize with README" (we already have one)
7. Click **"Create repository"**

GitHub will show you commands - **ignore them for now**.

---

### Step 2: Initialize Git in Your Workspace

```bash
cd ~/slam_uncertainty_ws

# Initialize git (if not already done)
git init

# Add all files
git add .

# Create first commit
git commit -m "Initial commit: Uncertainty-Aware SLAM implementation

- Real-time entropy computation
- Active exploration
- ECS metric logging
- Complete ROS2 Humble package
"
```

---

### Step 3: Connect to GitHub

Replace `YOUR_USERNAME` with your actual GitHub username:

```bash
# Add remote
git remote add origin https://github.com/YOUR_USERNAME/uncertainty-aware-slam.git

# Verify
git remote -v
```

---

### Step 4: Push to GitHub

```bash
# Push to main branch
git branch -M main
git push -u origin main
```

**Note:** GitHub will ask for your username and password. For password, you need a **Personal Access Token** (not your GitHub password).

---

### Step 5: Create Personal Access Token (If Needed)

If push fails asking for password:

1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Click **"Generate new token"** → **"Generate new token (classic)"**
3. Note: `ROS2 SLAM Project`
4. Expiration: `90 days` (or your choice)
5. Select scopes: Check **`repo`** (all sub-items)
6. Click **"Generate token"**
7. **COPY THE TOKEN** (you'll only see it once!)
8. Use this token as your password when pushing

---

### Alternative: Use SSH (Recommended)

**Setup SSH Key:**

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t ed25519 -C "your_email@example.com"
# Press Enter 3 times (default location, no passphrase)

# Copy public key
cat ~/.ssh/id_ed25519.pub
```

**Add to GitHub:**

1. Copy the output (starts with `ssh-ed25519`)
2. Go to GitHub → Settings → SSH and GPG keys
3. Click **"New SSH key"**
4. Title: `WSL Ubuntu`
5. Paste the key
6. Click **"Add SSH key"**

**Change remote to SSH:**

```bash
git remote set-url origin git@github.com:YOUR_USERNAME/uncertainty-aware-slam.git
git push -u origin main
```

---

### Step 6: Verify on GitHub

1. Go to `https://github.com/YOUR_USERNAME/uncertainty-aware-slam`
2. You should see all your files!
3. The README.md will display automatically

---

## What Gets Pushed

✅ **Included:**
- `src/uncertainty_slam/` - Your main package
- `src/slam_gmapping/` - Reference implementation
- All documentation (README, guides, etc.)
- Launch files, config files, scripts
- `.gitignore` (excludes build files)

❌ **Excluded (by .gitignore):**
- `build/` directory
- `install/` directory
- `log/` directory
- `__pycache__/` and compiled Python files
- Temporary files

---

## Future Updates

After making changes:

```bash
cd ~/slam_uncertainty_ws

# Check what changed
git status

# Add changed files
git add .

# Commit with message
git commit -m "Description of changes"

# Push to GitHub
git push
```

---

## Common Issues

### Issue: "Permission denied (publickey)"
**Solution:** Use HTTPS instead of SSH, or set up SSH key (see above)

### Issue: "Authentication failed"
**Solution:** Use Personal Access Token as password, not GitHub password

### Issue: "Repository not found"
**Solution:** Check repository name and your username in the URL

### Issue: Large files error
**Solution:** Bag files are excluded by .gitignore. If you need to include them:
```bash
git lfs install  # Install Git LFS first
git lfs track "*.bag"
git add .gitattributes
```

---

## Optional: Add a License Badge

After pushing, add this to the top of README.md:

```markdown
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
```

---

## Quick Reference

```bash
# One-time setup
cd ~/slam_uncertainty_ws
git init
git add .
git commit -m "Initial commit"
git remote add origin https://github.com/YOUR_USERNAME/uncertainty-aware-slam.git
git push -u origin main

# Future updates
git add .
git commit -m "Update: description"
git push
```

---

## Complete Example

```bash
# Navigate to workspace
cd ~/slam_uncertainty_ws

# Initialize (if not done)
git init

# Add everything
git add .

# Commit
git commit -m "Initial commit: Complete Uncertainty-Aware SLAM implementation"

# Add remote (REPLACE YOUR_USERNAME!)
git remote add origin https://github.com/YOUR_USERNAME/uncertainty-aware-slam.git

# Push
git branch -M main
git push -u origin main
```

---

**That's it!** Your project is now on GitHub. 🎉

You can share the link: `https://github.com/YOUR_USERNAME/uncertainty-aware-slam`
