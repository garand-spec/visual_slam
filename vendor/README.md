# Third-party dependencies

The three source dependencies are Git submodules pinned to the versions on the Jetson.
Their local customizations are stored in `patches/` and listed in `dependencies.json`.
The camera SDK, including the ARM64 vendor libraries and compatibility library, is included in `rervision_single_imu/`.

After a fresh clone, run from the project root:

```bash
git submodule update --init --recursive
for name in ORB_SLAM3 opencv_cuda opencv_contrib_cuda; do
    git -C "vendor/$name" apply --check "../patches/$name.patch" || exit 1
    git -C "vendor/$name" apply "../patches/$name.patch" || exit 1
done
```

Apply patches once, only to clean dependencies at the pinned revisions. Existing Jetson checkouts already contain these patches.
`ignore = dirty` keeps the parent working tree status quiet for these expected dependency changes; use `git -C vendor/<name> status` to inspect them.
When changing dependency source, update the matching patch with `git -C vendor/<name> diff --binary HEAD > vendor/patches/<name>.patch` from the project root.

Build the patched OpenCV and ORB-SLAM3 dependencies on the target device before running the root build scripts. Existing device builds are retained. Build products, datasets and runtime output are not uploaded. See the root README for environment and dependency paths.
Each dependency retains its upstream license; ORB-SLAM3 is GPLv3.
