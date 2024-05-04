#!/bin/bash

# Generates random points and then the convex hull and Delaunay triangulation.
POINTS_SIZE=100
BAZEL_ROOT=~/dev/hemesh

OBJ_DIR="${BAZEL_ROOT}/data/obj"
SVG_DIR="${BAZEL_ROOT}/data/svg"

POINTS_OBJ="${OBJ_DIR}/random_${POINTS_SIZE}_points.obj"
CONVEX_HULL_OBJ="${OBJ_DIR}/random_${POINTS_SIZE}_convex_hull.obj"
DELAUNAY_OBJ="${OBJ_DIR}/random_${POINTS_SIZE}_delaunay.obj"

CONVEX_HULL_SVG="${SVG_DIR}/random_${POINTS_SIZE}_convex_hull.svg"
DELAUNAY_SVG="${SVG_DIR}/random_${POINTS_SIZE}_delaunay.svg"

echo "Generate random points .obj file."
cd ${BAZEL_ROOT}; bazel run //tools:random_points_cli -- \
  --input_points_size=${POINTS_SIZE} \
  --output_points_obj="${POINTS_OBJ}"

echo "Generate the convex hull of the random 2D points."
cd ${BAZEL_ROOT}; bazel run //tools:convex_hull_2d_cli -- \
  --input_points_obj="${POINTS_OBJ}" \
  --output_convex_hull_obj="${CONVEX_HULL_OBJ}"

echo "Convert the convex hull .obj to .svg."
cd ${BAZEL_ROOT}; bazel run //tools:obj_to_svg_cli -- \
  --input_obj="${CONVEX_HULL_OBJ}" \
  --output_svg="${CONVEX_HULL_SVG}"

echo "Generate the Delaunay triangulation of the random 2D points."
cd ${BAZEL_ROOT}; bazel run //tools:delaunay_triangulator_cli -- \
  --input_points_obj="${POINTS_OBJ}" \
  --output_delaunay_obj="${DELAUNAY_OBJ}"

echo "Convert the Delaunay triangulation .obj to .svg."
cd ${BAZEL_ROOT}; bazel run //tools:obj_to_svg_cli -- \
  --input_obj="${DELAUNAY_OBJ}" \
  --output_svg="${DELAUNAY_SVG}"
