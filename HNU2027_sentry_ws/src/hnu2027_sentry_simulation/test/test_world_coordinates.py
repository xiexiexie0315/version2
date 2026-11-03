import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


class WorldCoordinateTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.geometry = json.loads((self.root / "config" / "tunnel_geometry.json").read_text())
        self.sdf = ET.parse(self.root / "resource" / "models" / "rmul_2027_3v3" / "model.sdf")

    def test_roof_centers_and_dimensions_match_cad_prior(self):
        expected = {item["id"]: item for item in self.geometry}
        for link in self.sdf.findall(".//link"):
            if not link.attrib["name"].endswith("_roof"):
                continue
            tunnel_id = link.attrib["name"].removesuffix("_roof")
            prior = expected[tunnel_id]
            pose = [float(value) for value in link.find("pose").text.split()]
            footprint = prior["roof_footprint"]
            self.assertAlmostEqual(pose[0], prior["center"][0], places=6)
            self.assertAlmostEqual(pose[1], prior["center"][1], places=6)
            self.assertAlmostEqual(pose[2], (prior["ceiling_z"] + prior["roof_top_z"]) / 2.0, places=6)
            dimensions = [float(value) for value in link.find("collision/geometry/box/size").text.split()]
            self.assertAlmostEqual(dimensions[0], footprint[1] - footprint[0], places=6)
            self.assertAlmostEqual(dimensions[1], footprint[3] - footprint[2], places=6)
            self.assertAlmostEqual(dimensions[2], prior["roof_top_z"] - prior["ceiling_z"], places=6)

    def test_world_references_the_2027_model(self):
        world = ET.parse(self.root / "resource" / "worlds" / "rmul_2027_3v3_world.sdf")
        self.assertEqual(world.findtext(".//include/uri"), "model://rmul_2027_3v3")


if __name__ == "__main__":
    unittest.main()
