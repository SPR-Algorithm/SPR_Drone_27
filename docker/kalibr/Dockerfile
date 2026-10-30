# This local base already contains the pinned Kalibr build and its build cache.
# No packages, source revisions, or network resources are fetched here.
ARG KALIBR_BASE_IMAGE=sha256:304c350836a49d8066c53a3cb34ff1c966b25715233318c870ff1fa7585bd9bf
FROM ${KALIBR_BASE_IMAGE}

COPY bsplines-cholmod.patch /opt/spr/bsplines-cholmod.patch
RUN cd /catkin_ws/src/kalibr \
    && test "$(git rev-parse HEAD)" = 1f60227442d25e36365ef5f72cd80b9666d73467 \
    && git apply --check /opt/spr/bsplines-cholmod.patch \
    && git apply /opt/spr/bsplines-cholmod.patch \
    && /bin/bash -lc 'source /catkin_ws/devel/setup.bash && cmake --build /catkin_ws/build/bsplines --target bsplines --parallel 2'

# A fresh process must import the real numerical backend, not just rosbag.
RUN /bin/bash -lc 'source /catkin_ws/devel/setup.bash && python3 -c "import bsplines; import kalibr_imu_camera_calibration.IccCalibrator"'

LABEL spr.kalibr.commit="1f60227442d25e36365ef5f72cd80b9666d73467" \
      spr.kalibr.patch="bsplines-cholmod.patch"
