#pragma once

#include <Geometry.h>

struct PoseGraph
{
    constexpr static int max_frames = 50;

    struct Frame
    {
        const char* name;
        int index;
        bool visited;
        Frame* parent;

        Frame(const char* name_, int index_) : name(name_), index(index_) {}
    };

    struct LookupResult
    {
        bool was_found;
        Geometry::Pose transform;
    };

    ~PoseGraph();

    LookupResult get_transform(const char* target_frame_id, const char* source_frame_id);

    bool add_transform(const char* target_frame_id, const char* source_frame_id, const Geometry::Pose& transform);

   private:
    int num_frames = 0;
    Frame* frames[max_frames];
    Geometry::Pose* transforms[max_frames][max_frames] = {NULL};

    Frame* get_frame(const char* frame_id);
};
