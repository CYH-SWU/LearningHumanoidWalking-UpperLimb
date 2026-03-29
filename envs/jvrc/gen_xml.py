import os
import sys

from dm_control import mjcf

import models

JVRC_DESCRIPTION_PATH = os.path.join(os.path.dirname(models.__file__), "jvrc_mj_description/xml/scene.xml")

WAIST_JOINTS = ["WAIST_Y", "WAIST_P", "WAIST_R"]
HEAD_JOINTS = ["NECK_Y", "NECK_R", "NECK_P"]
HAND_JOINTS = [
    "R_UTHUMB",
    "R_LTHUMB",
    "R_UINDEX",
    "R_LINDEX",
    "R_ULITTLE",
    "R_LLITTLE",
    "L_UTHUMB",
    "L_LTHUMB",
    "L_UINDEX",
    "L_LINDEX",
    "L_ULITTLE",
    "L_LLITTLE",
]
ARM_JOINTS = [
    "R_SHOULDER_P",
    "R_SHOULDER_R",
    "R_SHOULDER_Y",
    "R_ELBOW_P",
    "R_ELBOW_Y",
    "R_WRIST_R",
    "R_WRIST_Y",
    "L_SHOULDER_P",
    "L_SHOULDER_R",
    "L_SHOULDER_Y",
    "L_ELBOW_P",
    "L_ELBOW_Y",
    "L_WRIST_R",
    "L_WRIST_Y",
]
LEG_JOINTS = [
    "R_HIP_P",
    "R_HIP_R",
    "R_HIP_Y",
    "R_KNEE",
    "R_ANKLE_R",
    "R_ANKLE_P",
    "L_HIP_P",
    "L_HIP_R",
    "L_HIP_Y",
    "L_KNEE",
    "L_ANKLE_R",
    "L_ANKLE_P",
]

KEEP_ARM_JOINTS = [
    "R_SHOULDER_P",
    "R_ELBOW_P",
    "L_SHOULDER_P",
    "L_ELBOW_P"
]


def builder(export_path, config):
    print("Modifying XML model...")
    mjcf_model = mjcf.from_path(JVRC_DESCRIPTION_PATH)

    mjcf_model.model = "jvrc"

    # set njmax and nconmax
    mjcf_model.size.njmax = -1
    mjcf_model.size.nconmax = -1
    mjcf_model.statistic.meansize = 0.1
    mjcf_model.statistic.meanmass = 2

    # modify skybox
    for tx in mjcf_model.asset.texture:
        if tx.type == "skybox":
            tx.rgb1 = "1 1 1"
            tx.rgb2 = "1 1 1"

    # remove all collisions
    mjcf_model.contact.remove()

    # remove actuators except for leg joints
    for mot in mjcf_model.actuator.motor:
        if mot.joint.name not in LEG_JOINTS and mot.joint.name not in KEEP_ARM_JOINTS:
            mot.remove()

    # Set actuator force/torque limits (ctrlrange)
    for mot in mjcf_model.actuator.motor:
        if mot.joint.name in LEG_JOINTS:
            mot.ctrlrange = [-500, 500]   
        elif mot.joint.name in KEEP_ARM_JOINTS:
            mot.ctrlrange = [-200, 200]   

    # remove unused joints
    ALL_JOINT = WAIST_JOINTS + HEAD_JOINTS + HAND_JOINTS + ARM_JOINTS
    for joint in ALL_JOINT:
        if joint in KEEP_ARM_JOINTS:
            continue
        mjcf_model.find("joint", joint).remove()

    # set joint limits for kept arm joints
    for jnt_name in KEEP_ARM_JOINTS:
        joint = mjcf_model.find("joint", jnt_name)
        if "SHOULDER" in jnt_name:
            joint.range = [-0.8, 0.8]   # ±45°
        elif "ELBOW" in jnt_name:
            joint.range = [-1.57, 0]    # 0~90


    # remove existing equality
    mjcf_model.equality.remove()

    # collision geoms
    collision_geoms = [
        "R_HIP_R_S",
        "R_HIP_Y_S",
        "R_KNEE_S",
        "L_HIP_R_S",
        "L_HIP_Y_S",
        "L_KNEE_S",
    ]

    # remove unused collision geoms
    for body in mjcf_model.worldbody.find_all("body"):
        for idx, geom in enumerate(body.geom):
            geom.name = body.name + "-geom-" + repr(idx)
            if geom.dclass.dclass == "collision":
                if body.name not in collision_geoms:
                    geom.remove()

    # move collision geoms to different group
    mjcf_model.default.default["collision"].geom.group = 3

    # manually create collision geom for feet and arms
    mjcf_model.worldbody.find("body", "R_ANKLE_P_S").add(
        "geom", dclass="collision", size="0.1 0.05 0.01", pos="0.029 0 -0.09778", type="box",
        rgba=[1, 0, 0, 0.5]
    )
    mjcf_model.worldbody.find("body", "L_ANKLE_P_S").add(
        "geom", dclass="collision", size="0.1 0.05 0.01", pos="0.029 0 -0.09778", type="box",
        rgba=[1, 0, 0, 0.5]
    )
    
    mjcf_model.worldbody.find("body", "R_SHOULDER_P_S").add(
        "geom",
        name="R_upper_arm_collision",
        type="capsule",
        size=[0.05, 0.15],           
        pos=[0, 0, -0.15],           
        euler=[0, 0, 0],             
        dclass="collision",
        group=0,
        rgba=[1, 0, 0, 0.5]
    )
    
    mjcf_model.worldbody.find("body", "L_SHOULDER_P_S").add(
        "geom",
        name="L_upper_arm_collision",
        type="capsule",
        size=[0.05, 0.15],
        pos=[0, 0, -0.15],
        euler=[0, 0, 0],
        dclass="collision",
        group=0,
        rgba=[1, 0, 0, 0.5]
    )
    
    mjcf_model.worldbody.find("body", "R_ELBOW_P_S").add(
        "geom",
        name="R_forearm_collision",
        type="capsule",
        size=[0.04, 0.12],
        pos=[0, 0, -0.12],
        euler=[0, 0, 0],
        dclass="collision",
        group=0,
        rgba=[1, 0, 0, 0.5]
    )  
    
    mjcf_model.worldbody.find("body", "L_ELBOW_P_S").add(
        "geom",
        name="L_forearm_collision",
        type="capsule",
        size=[0.04, 0.12],
        pos=[0, 0, -0.12],
        euler=[0, 0, 0],
        dclass="collision",
        group=0,
        rgba=[1, 0, 0, 0.5]
    )
    
    try:
        mjcf_model.worldbody.find("body", "R_WRIST_Y_S").add(
            "geom", name="R_hand_collision", type="sphere", size=[0.04], pos=[0, 0, 0], dclass="collision", group=0,
            rgba=[1, 0, 0, 0.5]
        )
        mjcf_model.worldbody.find("body", "L_WRIST_Y_S").add(
            "geom", name="L_hand_collision", type="sphere", size=[0.04], pos=[0, 0, 0], dclass="collision", group=0,
            rgba=[1, 0, 0, 0.5]
        )
    except Exception:
        print("Warning: Could not add hand collision (wrist body missing)")
    
    # ignore collision
    mjcf_model.contact.add("exclude", body1="R_KNEE_S", body2="R_ANKLE_P_S")
    mjcf_model.contact.add("exclude", body1="L_KNEE_S", body2="L_ANKLE_P_S")

    mjcf_model.contact.add("exclude", body1="R_SHOULDER_P_S", body2="PELVIS_S")
    mjcf_model.contact.add("exclude", body1="L_SHOULDER_P_S", body2="PELVIS_S")


    # remove unused meshes
    meshes = [g.mesh.name for g in mjcf_model.find_all("geom") if g.type == "mesh" or g.type is None]
    for mesh in mjcf_model.find_all("mesh"):
        if mesh.name not in meshes:
            mesh.remove()

    # fix site pos
    mjcf_model.worldbody.find("site", "rf_force").pos = "0.03 0.0 -0.1"
    mjcf_model.worldbody.find("site", "lf_force").pos = "0.03 0.0 -0.1"

    # add box geoms
    if "boxes" in config and config["boxes"]:
        for idx in range(20):
            name = "box" + repr(idx + 1).zfill(2)
            mjcf_model.worldbody.add("body", name=name, pos=[0, 0, -0.2])
            mjcf_model.find("body", name).add(
                "geom", name=name, dclass="collision", group="0", size="1 1 0.1", type="box", material=""
            )

    # wrap floor geom in a body
    mjcf_model.find("geom", "floor").remove()
    mjcf_model.worldbody.add("body", name="floor")
    mjcf_model.find("body", "floor").add("geom", name="floor", type="plane", size="0 0 0.25", material="groundplane")

    # export model
    mjcf.export_with_assets(mjcf_model, out_dir=export_path, precision=5)
    path_to_xml = os.path.join(export_path, mjcf_model.model + ".xml")
    print("Exporting XML model to ", path_to_xml)
    return


if __name__ == "__main__":
    builder(sys.argv[1], config={})
