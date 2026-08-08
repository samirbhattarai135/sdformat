/*
 * Copyright 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// Measures what resolvePose costs today, and what a lowest-common-ancestor
// lookup would cost on the same graphs.
//
// resolvePose currently resolves both frames to the root of the graph and
// composes one with the inverse of the other. Resolving to the lowest common
// ancestor instead would skip the shared prefix, but adds an LCA lookup, and
// gz::math::graph::LowestCommonAncestor allocates an unordered_set plus a
// vector per call. Whether that trade wins depends on how deep the graph is,
// so measure both sides before changing anything.
//
// See https://github.com/gazebosim/sdformat/issues/1692

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gz/math/graph/GraphAlgorithms.hh>

#include "sdf/Model.hh"
#include "sdf/Root.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "test_config.hh"

namespace
{
/// \brief Build a model nested _depth levels deep. Each level carries a
/// rotation so the pose composition is not trivially cheap.
/// \param[in] _depth Number of nested models below the top level.
/// \return SDF text for the model.
std::string NestedModel(int _depth)
{
  std::string sdf =
      "<?xml version='1.0'?><sdf version='1.9'><model name='m0'>"
      "<link name='L'/>";
  for (int i = 1; i <= _depth; ++i)
  {
    sdf += "<model name='m" + std::to_string(i) + "'>"
           "<pose>1 0 0 -1.5707 0 0</pose>"
           "<link name='L'/>";
  }
  for (int i = 0; i < _depth; ++i)
    sdf += "</model>";
  sdf += "</model></sdf>";
  return sdf;
}

/// \brief Name of the deepest link, e.g. "m1::m2::L".
std::string DeepLinkName(int _depth)
{
  std::string name;
  for (int i = 1; i <= _depth; ++i)
    name += "m" + std::to_string(i) + "::";
  return name + "L";
}

constexpr int kIterations = 20000;
}  // namespace

/////////////////////////////////////////////////
TEST(FrameSemanticsPerformance, ResolvePoseVersusLowestCommonAncestor)
{
  std::printf("\n%-7s %14s %14s %14s\n",
      "depth", "resolvePose", "LCA lookup", "LCA/resolve");

  for (const int depth : {1, 2, 4, 8, 16})
  {
    sdf::Root root;
    ASSERT_TRUE(root.LoadSdfString(NestedModel(depth)).empty()) << depth;
    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model) << depth;

    auto ownedGraph = std::make_shared<sdf::PoseRelativeToGraph>();
    sdf::ScopedGraph<sdf::PoseRelativeToGraph> graph(ownedGraph);
    ASSERT_TRUE(sdf::buildPoseRelativeToGraph(graph, model).empty()) << depth;
    graph = graph.ChildModelScope(model->Name());

    const std::string deep = DeepLinkName(depth);
    ASSERT_EQ(1u, graph.Count(deep)) << deep;
    ASSERT_EQ(1u, graph.Count("L"));

    const auto deepId = graph.VertexIdByName(deep);
    const auto shallowId = graph.VertexIdByName("L");

    // Current behaviour: both frames resolved to the root, then composed.
    gz::math::Pose3d pose;
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i)
    {
      sdf::resolvePose(pose, graph, deepId, shallowId);
    }
    const auto resolveNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start).count() / kIterations;

    // What an LCA-based implementation would have to pay before composing
    // anything at all.
    start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i)
    {
      auto lca = gz::math::graph::LowestCommonAncestor(
          graph.Graph(), deepId, shallowId);
      ASSERT_NE(gz::math::graph::kNullId, lca) << depth;
    }
    const auto lcaNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start).count() / kIterations;

    std::printf("%-7d %12lldns %12lldns %13.2f\n", depth,
        static_cast<long long>(resolveNs), static_cast<long long>(lcaNs),
        resolveNs > 0 ? static_cast<double>(lcaNs) / resolveNs : 0.0);
  }

  std::printf(
      "\nIf the LCA lookup alone costs a large fraction of a whole "
      "resolvePose,\nresolving to the LCA cannot pay for itself at that "
      "depth.\n\n");
}
