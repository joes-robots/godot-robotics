#include "mujoco_verify.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <mujoco/mjxmacro.h>

using namespace godot;

namespace {

// Two numbers count as equal if they differ by less than this fraction of the
// larger one (or by less than this outright, for numbers smaller than 1).
const double TOLERANCE = 1e-6;
// The same, for joint positions after simulating. Small differences grow as a
// simulation runs, so this is looser.
const double TRAJECTORY_TOLERANCE = 1e-4;

bool close_enough(double p_a, double p_b, double p_tolerance)
{
    if (std::isnan(p_a) || std::isnan(p_b)) {
        return std::isnan(p_a) && std::isnan(p_b);
    }
    const double scale = std::fmax(1.0, std::fmax(std::fabs(p_a), std::fabs(p_b)));
    return std::fabs(p_a - p_b) <= p_tolerance * scale;
}

void add_difference(Array &r_differences, const String &p_field, const String &p_message, int64_t p_count, double p_max_difference)
{
    Dictionary entry;
    entry["field"] = p_field;
    entry["message"] = p_message;
    entry["count"] = p_count;
    entry["max_difference"] = p_max_difference;
    r_differences.push_back(entry);
}

// A number to three significant digits, for messages.
String short_number(double p_value)
{
    char text[32];
    std::snprintf(text, sizeof(text), "%.3g", p_value);
    return String(text);
}

bool starts_with(const char *p_text, const char *p_prefix)
{
    return std::strncmp(p_text, p_prefix, std::strlen(p_prefix)) == 0;
}

bool ends_with(const char *p_text, const char *p_suffix)
{
    const size_t length = std::strlen(p_text);
    const size_t suffix_length = std::strlen(p_suffix);
    return length >= suffix_length && std::strcmp(p_text + length - suffix_length, p_suffix) == 0;
}

// Fields left out of the comparison; see the header for why.
bool is_skipped_field(const char *p_name)
{
    return starts_with(p_name, "name_") || std::strcmp(p_name, "names") == 0 || std::strcmp(p_name, "names_map") == 0
            || std::strcmp(p_name, "paths") == 0 || std::strstr(p_name, "bvh") != nullptr || starts_with(p_name, "oct_");
}

bool is_skipped_size(const char *p_name)
{
    static const char *const skipped[] = { "nnames", "nnames_map", "npaths", "nbvh", "nbvhstatic", "nbvhdynamic",
        "noct", "narena", "nbuffer" };
    for (const char *name : skipped) {
        if (std::strcmp(p_name, name) == 0) {
            return true;
        }
    }
    return false;
}

// Compares one array field. p_count_a and p_count_b are the arrays' lengths;
// p_columns is how many numbers make up one entry (3 for a position, say).
template <typename T>
void compare_field(Array &r_differences, const char *p_name, const T *p_a, const T *p_b, size_t p_count_a, size_t p_count_b, int p_columns)
{
    if (is_skipped_field(p_name)) {
        return;
    }
    if (p_count_a != p_count_b) {
        // The models have different numbers of something; that is reported
        // once, among the sizes, not again for every array it affects.
        return;
    }

    // A rotation q and its negative -q are the same rotation.
    const bool is_quaternion = p_columns == 4 && ends_with(p_name, "quat");

    int64_t differing = 0;
    double max_difference = 0.0;
    for (size_t i = 0; i < p_count_a; i++) {
        if constexpr (std::is_floating_point_v<T>) {
            double a = p_a[i];
            double b = p_b[i];
            if (is_quaternion) {
                // Compare the whole quaternion both ways round and keep the better.
                const size_t row = i - i % 4;
                double same = 0.0, flipped = 0.0;
                for (int k = 0; k < 4; k++) {
                    same += std::fabs(double(p_a[row + k]) - double(p_b[row + k]));
                    flipped += std::fabs(double(p_a[row + k]) + double(p_b[row + k]));
                }
                if (flipped < same) {
                    b = -b;
                }
            }
            if (!close_enough(a, b, TOLERANCE)) {
                differing++;
                max_difference = std::fmax(max_difference, std::fabs(a - b));
            }
        } else {
            if (p_a[i] != p_b[i]) {
                differing++;
            }
        }
    }

    if (differing > 0) {
        String message = itos(differing) + String(" of ") + itos(int64_t(p_count_a)) + String(differing == 1 ? " values differs" : " values differ");
        if (max_difference > 0) {
            message += String(", by up to ") + short_number(max_difference);
        }
        add_difference(r_differences, p_name, message + String("."), differing, max_difference);
    }
}

// Names of one kind of element. An element the original left unnamed is not
// compared: in a model built from nodes, every element has a name.
void compare_names(Array &r_differences, const mjModel *p_a, const mjModel *p_b, int p_type, int p_count_a, int p_count_b, const char *p_field)
{
    if (p_count_a != p_count_b) {
        return;
    }
    int64_t differing = 0;
    String first;
    for (int i = 0; i < p_count_a; i++) {
        const char *a = mj_id2name(p_a, p_type, i);
        const char *b = mj_id2name(p_b, p_type, i);
        if (!a || !a[0]) {
            continue;
        }
        if (!b || std::strcmp(a, b) != 0) {
            if (differing == 0) {
                first = String("'") + String::utf8(a) + String("' is '") + String::utf8(b ? b : "") + String("'");
            }
            differing++;
        }
    }
    if (differing > 0) {
        add_difference(r_differences, p_field, itos(differing) + String(differing == 1 ? " name differs: " : " names differ, the first: ") + first + String("."), differing, 0.0);
    }
}

} // namespace

// The comparison is driven by lists that MuJoCo publishes as "X macros": a
// macro such as MJMODEL_POINTERS expands to one X(type, name, rows, columns)
// per array in mjModel. Defining X before using the list turns it into one
// piece of code per field.
//
// Some of the row and column counts are themselves model fields (written
// MJ_M(n)). MJMODEL_POINTERS_PREAMBLE(m) declares them as local variables for
// one model, m, which is why each field's size is worked out inside a small
// function taking the model. Most of those variables go unused in any one
// function, hence the warning being turned off here.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

Array mujoco_compare_models(const mjModel *p_original, const mjModel *p_other, int p_steps)
{
    Array differences;
    const mjModel *a = p_original;
    const mjModel *b = p_other;

    // 1. Sizes: how many bodies, joints, geoms and so on.
#define X(name) \
    if (!is_skipped_size(#name) && a->name != b->name) { \
        add_difference(differences, #name, String("The original has ") + itos(int64_t(a->name)) + String(", the import has ") + itos(int64_t(b->name)) + String("."), 1, 0.0); \
    }
    MJMODEL_SIZES
#undef X

    // 2. Options.
    if (std::memcmp(&a->opt, &b->opt, sizeof(mjOption)) != 0) {
        add_difference(differences, "opt", "The physics options differ.", 1, 0.0);
    }

    // 3. Every array field.
#define X(type, name, rows, columns) \
    { \
        auto row_count = [](const mjModel *m) -> size_t { MJMODEL_POINTERS_PREAMBLE(m); return size_t(m->rows); }; \
        auto column_count = [](const mjModel *m) -> size_t { MJMODEL_POINTERS_PREAMBLE(m); return size_t(columns); }; \
        compare_field<type>(differences, #name, a->name, b->name, row_count(a) * column_count(a), row_count(b) * column_count(b), int(column_count(a))); \
    }
    MJMODEL_POINTERS
#undef X

    // 4. Names, where the original has them.
    compare_names(differences, a, b, mjOBJ_BODY, a->nbody, b->nbody, "body names");
    compare_names(differences, a, b, mjOBJ_JOINT, a->njnt, b->njnt, "joint names");
    compare_names(differences, a, b, mjOBJ_GEOM, a->ngeom, b->ngeom, "geom names");

    // 5. Behaviour: simulate both from their initial state and compare where
    // the joints end up. Only meaningful if they have the same joints.
    if (p_steps > 0 && a->nq == b->nq && a->nv == b->nv && a->nu == b->nu) {
        mjData *data_a = mj_makeData(a);
        mjData *data_b = mj_makeData(b);
        for (int i = 0; i < p_steps; i++) {
            mj_step(a, data_a);
            mj_step(b, data_b);
        }
        int64_t differing = 0;
        double max_difference = 0.0;
        for (int i = 0; i < a->nq; i++) {
            if (!close_enough(data_a->qpos[i], data_b->qpos[i], TRAJECTORY_TOLERANCE)) {
                differing++;
                max_difference = std::fmax(max_difference, std::fabs(data_a->qpos[i] - data_b->qpos[i]));
            }
        }
        if (differing > 0) {
            add_difference(differences, "simulation", String("After ") + itos(p_steps) + String(" steps, ") + itos(differing) + String(" of ")
                    + itos(int64_t(a->nq)) + String(" joint positions differ, by up to ") + short_number(max_difference) + String("."), differing, max_difference);
        }
        mj_deleteData(data_a);
        mj_deleteData(data_b);
    }

    return differences;
}

#pragma GCC diagnostic pop
