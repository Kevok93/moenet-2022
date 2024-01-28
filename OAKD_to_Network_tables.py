from pathlib import Path

import Network_Tables_Sender
import fnmatch
import os
from random import randint
import time
from multiprocessing import Process, Queue
import sys

import pipelines.apriltag_pipeline
import pipelines.object_pipeline
from pipelines.pipeline import Pipeline


def file_saver(path, q: Queue):
    path = Path(path)

    current_count = 0
    session_name = randint(1,1000000000)
    while True:
        img = q.get()
        if path.exists() and img is not None:
            try:
                count = len(fnmatch.filter(os.listdir(path), '*.*'))
                if count < 6000 and current_count < 1200:
                    current_count += 1
                    #cv2.imwrite(str(path / f"{session_name}_{current_count}.png"), img)
                else:
                    pass
                    #time.sleep(10)
            except Exception as e:
                print(e)
        time.sleep(0.5)    


def main(mode = 'obj', mxid = None):
    # 0 - Camera on back for april tag detection, 1 - Camera on front for objet detection
    # io_proc = None
    try:
        Network_Tables_Sender.init('127.0.0.1')
        pipeline: Pipeline = None
        config: dict = {
            "device_id": mxid
        }
        if mode == 'obj':
            pipeline = pipelines.object_pipeline.ObjectPipeline()
        if mode == 'tag':
            pipeline = pipelines.apriltag_pipeline.ApriltagPipeline()
            config = pipelines.apriltag_pipeline.default_config() | config
        pipeline.start(config)
    except KeyboardInterrupt:
        raise
    except Exception as e:
        print(e)
    # finally:
    #     if io_proc is not None:
    #         io_proc.terminate()

# Hybrid mode - tag first, object detection
if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) >= 2 else 'obj'
    mxid = sys.argv[2] if len(sys.argv) >= 3 else None
    (mxid1, mxid2) = sys.argv[2:4] if len(sys.argv) >= 4 else (None,None)

    tag_detection_on = 0

    while True:
        try:
            if mode == 'hybrid':
                if not tag_detection_on:
                    try:
                        io_proc = Process(
                            target=main,
                            args=('tag', mxid1),
                            daemon=True
                        )
                        io_proc.start()
                        tag_detection_on = 1
                    except:
                        tag_detection_on = 0
                        pass
                try:
                    main('obj', mxid2)
                except:
                    pass        
            else:
                main(mode, mxid)
        except Exception as e:
            print(e)