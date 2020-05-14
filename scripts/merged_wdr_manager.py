import grpc
import image_pb2
import image_pb2_grpc
import numpy as np

# options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
#           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
#           ] # Message size is up to 10MB

from merge_far_near_images import MergeFarNearImage


class MergedWDRImageManager:
    def __init__(self, options):
        self.channel = grpc.insecure_channel('localhost:50051',
                                             options=options)
        self.rgb_img = None
        self.depth_img = None
        self.depth_image_merger = MergeFarNearImage()  # MERGE_QUEUE_SIZE = 5

    def update_depth_range1(self, response):
        w = response.image_depth_range1.width
        h = response.image_depth_range1.height
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_depth_range1.data, np.uint16)
        self.img_depth_range1 = img_np.reshape(h, w)
        return True

    def update_depth_range2(self, response):
        w = response.image_depth_range2.width
        h = response.image_depth_range2.height
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_depth_range2.data, np.uint16)
        self.img_depth_range2 = img_np.reshape(h, w)
        return True

    def update(self):
        stub = image_pb2_grpc.ImageServiceStub(self.channel)
        response = stub.SendWDRImage(image_pb2.ImageRequest())
        status = self.update_depth_range1(response)
        status &= self.update_depth_range2(response)
        return status

    def get_merger_result(self):
        if self.update():
            return self.depth_image_merger.update(
                self.img_depth_range1, self.img_depth_range2
            )

        else:
            return False

    @property
    def filtered_fused_depth(self):
        #count = 0
        while not self.get_merger_result():
            #count += 1
            pass
        return self.depth_image_merger.merged_depth_image
