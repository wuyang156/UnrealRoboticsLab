# Create M_MjLidarIsmPoint: the per-instance-colored material used by the
# MjLidarPointCloudViz InstancedMesh backend.
#
# UMjLidarPointCloudViz writes each point's linear RGB into Per-Instance
# Custom Data slots 0..2 (see RebuildInstancedMesh). UE 5.7 has no
# SetInstanceColor shortcut, so the material must read those slots itself:
# this script wires three MaterialExpressionPerInstanceCustomData nodes
# (indices 0/1/2) through AppendVector into Base Color and Emissive Color.
#
# Run ONCE with the editor open, from the Output Log's Python console:
#
#     exec(open(r"D:/UEProject/wuyang_lab/Plugins/UnrealRoboticsLab/Scripts/create_lidar_viz_material.py").read())
#
# Then assign the asset to the component's PointMaterial property.

import unreal

ASSET_PATH = "/UnrealRoboticsLab/Sensors/M_MjLidarIsmPoint"


def _set_custom_data_index(expression, index):
    """Sets the node's Custom Data Index, tolerating property renames."""
    for name in ("custom_data_index", "output_index"):
        try:
            expression.set_editor_property(name, index)
            return
        except Exception:
            continue
    unreal.log_warning(
        "M_MjLidarIsmPoint: could not set the per-instance custom data index "
        "on a node; set it to %d by hand in the material editor." % index
    )


def main():
    editor = unreal.EditorAssetLibrary
    mel = unreal.MaterialEditingLibrary

    if editor.does_asset_exist(ASSET_PATH):
        unreal.log_warning("M_MjLidarIsmPoint: %s already exists - recreating it." % ASSET_PATH)
        editor.delete_asset(ASSET_PATH)

    editor.make_directory("/UnrealRoboticsLab/Sensors")

    material = editor.create_asset(
        "M_MjLidarIsmPoint",              # asset name
        "/UnrealRoboticsLab/Sensors",     # package path (plugin content mount)
        unreal.Material,                  # class (kept for older API signatures)
        unreal.MaterialFactoryNew(),      # factory
    )
    if material is None:
        unreal.log_error("M_MjLidarIsmPoint: create_asset failed; is 'Show Plugin Content' on and the URLab plugin loaded?")
        return

    # One PerInstanceCustomData node per RGB channel (slots written by the component).
    channels = []
    for slot in range(3):
        node = mel.create_material_expression(
            material, unreal.MaterialExpressionPerInstanceCustomData, -700, 150 * slot)
        _set_custom_data_index(node, slot)
        channels.append(node)

    # R and G -> RG, then RG + B -> RGB.
    append_rg = mel.create_material_expression(material, unreal.MaterialExpressionAppendVector, -450, 75)
    mel.connect_material_expressions(channels[0], "", append_rg, "A")
    mel.connect_material_expressions(channels[1], "", append_rg, "B")

    append_rgb = mel.create_material_expression(material, unreal.MaterialExpressionAppendVector, -250, 75)
    mel.connect_material_expressions(append_rg, "", append_rgb, "A")
    mel.connect_material_expressions(channels[2], "", append_rgb, "B")

    mel.connect_material_property(append_rgb, "", unreal.MaterialProperty.MP_BASE_COLOR)
    mel.connect_material_property(append_rgb, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

    mel.reconstruct_material(material)

    if editor.save_asset(ASSET_PATH):
        unreal.log("M_MjLidarIsmPoint: created %s (Custom Data 0..2 -> BaseColor/Emissive). "
                   "Assign it to UMjLidarPointCloudViz.PointMaterial." % ASSET_PATH)
    else:
        unreal.log_error("M_MjLidarIsmPoint: material created but saving failed.")


main()
