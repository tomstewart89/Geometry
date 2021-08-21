#include <PoseGraph.h>

using namespace Geometry;
using namespace BLA;

PoseGraph::~PoseGraph()
{
    for (int i = 0; i < max_frames; ++i)
    {
        if (frames[i])
        {
            delete frames[i];
        }
        for (int j = 0; j < max_frames; ++j)
        {
            if (transforms[i][j])
            {
                delete transforms[i][j];
            }
        }
    }
}

PoseGraph::Frame* PoseGraph::get_frame(const char* frame_id)
{
    for (int i = 0; i < num_frames; ++i)
    {
        if (strcmp(frames[i]->name, frame_id) == 0)
        {
            return frames[i];
        }
    }

    frames[num_frames] = new Frame(frame_id, num_frames);

    return frames[num_frames++];
}

bool PoseGraph::add_transform(const char* target_frame_id, const char* source_frame_id, const Pose& transform)
{
    if (get_transform(target_frame_id, source_frame_id).was_found)
    {
        return false;
    }

    Frame* target_frame = get_frame(target_frame_id);
    Frame* source_frame = get_frame(source_frame_id);

    transforms[target_frame->index][source_frame->index] = new Pose(transform);
    transforms[source_frame->index][target_frame->index] = new Pose(transform.inv());

    return true;
}

PoseGraph::LookupResult PoseGraph::get_transform(const char* target_frame_id, const char* source_frame_id)
{
    for (int i = 0; i < num_frames; ++i)
    {
        frames[i]->visited = false;
    }

    Frame* target_frame = get_frame(target_frame_id);
    Frame* source_frame = get_frame(source_frame_id);

    if (target_frame == source_frame)
    {
        return {true, Pose(Identity<3>(), {0, 0, 0})};
    }

    Frame* stack[max_frames];

    int stack_size = 0;
    stack[stack_size++] = target_frame;

    bool found_source_frame = false;

    while (stack_size && !found_source_frame)
    {
        Frame* frame = stack[--stack_size];
        frame->visited = true;

        for (int i = 0; i < num_frames; ++i)
        {
            if (transforms[frame->index][i] && !frames[i]->visited)
            {
                stack[stack_size++] = frames[i];
                frames[i]->parent = frame;

                if (frames[i] == source_frame)
                {
                    found_source_frame = true;
                    break;
                }
            }
        }
    }

    if (found_source_frame)
    {
        Pose T_source_target(Identity<3>(), {0, 0, 0});
        Frame* frame = source_frame;

        while (true)
        {
            if (frame == target_frame)
            {
                break;
            }

            T_source_target = *transforms[frame->index][frame->parent->index] * T_source_target;
            frame = frame->parent;
        }

        return {true, T_source_target.inv()};
    }
    else
    {
        return {false, Pose()};
    }
}
