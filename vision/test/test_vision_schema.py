"""Unit tests for the vision schema (no model/network needed)."""

import unittest

from vision.vision_schema import Detection, Scene


class TestScene(unittest.TestCase):
    def test_find(self):
        scene = Scene(
            image_width=640,
            image_height=480,
            objects=[
                Detection(name="cup", confidence=0.92, bbox=[1, 2, 3, 4]),
                Detection(name="person", confidence=0.8, bbox=[5, 6, 7, 8]),
            ],
            source="yolo",
        )
        cups = scene.find("cup")
        self.assertEqual(len(cups), 1)
        self.assertEqual(cups[0].name, "cup")
        self.assertEqual(scene.find("CUP"), cups)
        self.assertEqual(len(scene.find("none")), 0)

    def test_to_json(self):
        scene = Scene(
            objects=[Detection(name="cup", confidence=0.92, bbox=[1, 2, 3, 4])],
            source="yolo",
            observed_at=123.0,
        )
        data = scene.to_dict()
        self.assertEqual(data["objects"][0]["name"], "cup")
        self.assertEqual(data["observed_at"], 123.0)
        self.assertIn("cup", scene.to_json())


if __name__ == "__main__":
    unittest.main()
