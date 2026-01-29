# Copyright (C) 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from io import BytesIO
import time
import datetime
import json
import logging
import os
import numpy as np
from threading import Lock
from typing import Set
from PIL import Image


logger = logging.getLogger('FRAME_SELECTOR')
logger.setLevel(logging.DEBUG)

class FrameSave:
    def __init__(self, *args, **kwargs):
        try:
            logger.info("Initializing FrameSelector via gvapython extension...")

            self.get_env_variables()
            self.interested: list = kwargs.get("interested")

            self.output_dir = "/tmp/best_frames"
            self.frame_id = 1
            self.skip_n_frames = 30
            self._lock = Lock()
            os.makedirs(self.output_dir, exist_ok=True)

            # Whether to save full frame instead of ROI crop
            self.save_full_frame = True

            self.frame_count = 0

            logger.info("FrameSelector initialized successfully.")

        except Exception as e:
            logger.error(f"Failed to initialize FrameSelector: {str(e)}")
            raise

    def __del__(self):
        # On pipeline stop, flush any remaining best crops to disk
        print("deleting resources")
        # try:
        #     for oid in list(self.best_frames.keys()):
        #         self._save_best_and_cleanup(oid)
        # except Exception:
        #     pass

    def get_env_variables(self):
        try:
            print("Getting environment variables for FrameSelector...")

        except ValueError:
            logger.error("Port value should be an integer.")
            raise Exception("Port value should be an integer.")

    def process(self, frame):

        with frame.data() as np_frame:
            video_info = frame.video_info()
            fmt = video_info.to_caps().get_structure(0).get_value('format')


            file_name = f"frame_{self.frame_id}.jpg"
            self.save_image(np_frame, file_name, fmt)

            self.frame_id += 1

            return True



    def save_image(self, image_data, image_filename, image_format):
        # Ensure output directory exists
        output_dir = "/tmp/best_frames"
        os.makedirs(output_dir, exist_ok=True)

        # Convert BGR/BGRx/BGRA to RGB if needed
        if image_format in ["BGR", "BGRx", "BGRA"]:
            image_data = image_data[:, :, 2::-1]

        # Create PIL image
        image = Image.fromarray(image_data)

        # Build full path
        full_path = os.path.join(output_dir, image_filename)

        try:
            # Save image as JPEG with quality 85
            image.save(full_path, format="JPEG", quality=85)
            logger.info(f"Image saved successfully at {full_path}")
        except Exception as e:
            logger.error(f"Failed to save image {full_path}: {e}")

    def save_metadata(self, metadata):
        metadata_output_path = "/tmp/frames_metadata.json"

        try:
            # Write to JSON file
            with self._lock:
                with open(metadata_output_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(metadata))
                    f.write("\n")

                logger.info(f"Metadata saved successfully at {metadata_output_path}")

        except Exception as e:
            logger.error(f"Failed to save metadata: {e}")

    def get_gva_metadata(self, messages:list) -> dict:
        """Takes a list of frame meta messages, loads them as a JSON and
        updates the metadata dict with the loaded JSON.
        """

        metadata: dict = {}
        for message in messages:
            message_json = json.loads(message)
            metadata.update(message_json)

        return metadata
