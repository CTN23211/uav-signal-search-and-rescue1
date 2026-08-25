# Updating the existing GitHub repository to V3

Because V3 reorganizes the directory structure, copying it on top of V2 without deleting old tracked files can leave stale V2 directories in the repository.

Recommended GitHub Desktop workflow:

1. Make sure the existing repository has already been pushed / backed up.
2. In GitHub Desktop choose **Repository -> Show in Explorer**.
3. In that local repository directory, delete the old **visible repository files and folders**. Do not delete the hidden `.git` directory.
4. Extract the V3 ZIP.
5. Open the extracted `autonomous-uav-ugv-cooperative-system/` directory.
6. Copy **all contents inside that directory** into the existing GitHub local repository directory.
7. Return to GitHub Desktop. It should show additions, modifications and deletions.
8. Review the diff.
9. Commit with a message such as:

```text
Restructure repository for V3 end-to-end search stack
```

10. Push.

After the push, check the GitHub web page and confirm that `uav/`, `ground_station/`, `third_party/`, `config/` and `docs/` are visible at repository root.
