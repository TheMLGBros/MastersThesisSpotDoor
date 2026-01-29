import cv2
import torch
from typing import Tuple
import torchvision

names = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
            'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
            'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
            'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]


class ImgDetector():
    def __init__(self):
        print("INIT")
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        
    
    def get_grab_point(self, img, confidence: float = 0.05, plot = False) -> list[Tuple[int, int]]:
        #cv2.imwrite("./test.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        img_tensor = torch.from_numpy(img).float() / 255.0
        # Change from (H, W, C) to (C, H, W)
        img_tensor = img_tensor.permute(2, 0, 1)
        # Add batch dimension
        img_tensor = img_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            predictions = self.model(img_tensor)

        middle_of_balls = []
        pred = predictions[0]
        boxes = pred['boxes'].cpu().numpy()
        scores = pred['scores'].cpu().numpy()
        labels = pred['labels'].cpu().numpy()
        mask = (scores > confidence) & (labels == names.index('sports ball'))
        
        for box in boxes[mask]:
            x1, y1, x2, y2 = box
            middle_of_balls.append(((x1 + x2) / 2, (y1 + y2) / 2))

        return middle_of_balls
            



#cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
#cv2.putText(img, label, (x1, y1 - 10), 
#            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#cv2.imshow("Detections", img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()