#!/bin/sh

# Check if there are unstaged changes.
# The `git diff-index` command returns a non-zero status if there are any changes.
#if ! git diff-index --quiet HEAD --; then
if [ -n "$(git status --porcelain)" ]; then
  echo "Changes detected. Automatically staging and committing..."
  git add .
  git commit -m "Auto-commit from sync script."
  echo "Changes committed successfully."
fi

# Check for new local commits on the main branch.
# This command compares the local main branch with the remote origin/main.
if git log origin/main..main --oneline | grep .; then
  echo "New local commits found. Pushing to origin/main..."
  git push origin main
  if [ $? -eq 0 ]; then
    echo "Push successful."
  else
    echo "Push failed. Please check for conflicts and try again."
    exit 1
  fi
else
  echo "No new local commits. Pulling from origin/main..."
  git pull origin main
  if [ $? -eq 0 ]; then
    echo "Pull successful."
  else
    echo "Pull failed. Please resolve conflicts and try again."
    exit 1
  fi
fi
