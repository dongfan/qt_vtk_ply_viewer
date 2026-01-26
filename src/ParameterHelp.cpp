#include "ParameterHelp.h"

HelpContent ParameterHelpDB::empty_ = {
    "No Help",
    "No description available.",
    "",
    ""
};

HelpDB ParameterHelpDB::db_ = {

    // ----------------------------
    // Voxel
    // ----------------------------
    {
        "voxel_mm",
        {
            {
                AppMode::Pick,
                {
                    "Voxel Size (mm)",
                    "Downsampling grid size for faster processing.",
                    "0.5 ~ 2.0 mm",
                    "Smaller voxel increases processing time."
                }
            },
            {
                AppMode::Inspection,
                {
                    "Voxel Size (mm)",
                    "Downsampling grid size that defines spatial resolution of measurement.",
                    "Voxel ¡Â (Target tolerance / 2)",
                    "Voxel size becomes the lower bound of measurable precision.\n"
                    "If voxel is 0.5mm, sub-0.5mm accuracy is impossible regardless of algorithm."
                }
            }
        }
    },

    // ----------------------------
    // Outlier Radius
    // ----------------------------
    {
        "outlier_radius",
        {
            {
                AppMode::Pick,
                {
                    "Outlier Radius (mm)",
                    "Radius used to remove isolated points.",
                    "2 ~ 10 mm",
                    "Large radius keeps more noise."
                }
            },
            {
                AppMode::Inspection,
                {
                    "Outlier Radius (mm)",
                    "Defines neighborhood radius used to decide whether a point is noise.",
                    "Radius ¡Ã 2 ¡¿ point spacing",
                    "Aggressive outlier removal can shift edges and bias dimension results.\n"
                    "Critical edges should be verified after filtering."
                }
            }
        }
    },

    {
        "outlier_min_neighbors",
        {
            {
                AppMode::Inspection,
                {
                    "Minimum Neighbors",
                    "Minimum number of neighbors required to keep a point.",
                    "3 ~ 8 (depends on surface density)",
                    "High MinN removes sparse but valid edge points,\n"
                    "leading to systematic underestimation of dimensions."
                }
            }
        }
    },
    {
        "plane_threshold",
        {
            {
                AppMode::Inspection,
                {
                    "Plane Threshold (mm)",
                    "Maximum distance for a point to be considered part of a plane.",
                    "Threshold ¡Â expected surface flatness",
                    "Large threshold hides warpage and flatness defects.\n"
                    "Too small threshold may reject valid surface points."
                }
            }
        }
    },
    {
        "smoothing_sigma",
        {
            {
                AppMode::Inspection,
                {
                    "Smoothing Strength",
                    "Applies spatial smoothing to point cloud or surface.",
                    "Use minimal smoothing only for noise stabilization",
                    "Smoothing alters true geometry and reduces edge sharpness.\n"
                    "Never apply smoothing before final dimensional measurement."
                }
            }
        }
    }

};

const HelpContent& ParameterHelpDB::get(const QString& key, AppMode mode)
{
    if (!db_.contains(key))
        return empty_;

    const auto& modeMap = db_[key];
    if (!modeMap.contains(mode))
        return empty_;

    return modeMap[mode];
}
