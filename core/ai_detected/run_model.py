
import os
import numpy as np
import torch
from PIL import Image
import torchvision
import matplotlib.pyplot as plt
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision import transforms as T
# from dataset import PennFudanDataset, get_transform
import cv2

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')


class LoadAIModel:

    def __init__(self, modelpath):
        num_class = 2
        self.model = None
        self.model = self.get_model_instance_segmentation(num_class)
        self.model.load_state_dict(torch.load(f"{modelpath}"))

    def model_state(self):
        return self.model

    def get_model_instance_segmentation(self, num_classes):
        # load an instance segmentation model pre-trained pre-trained on COCO
        model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)

        # get number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        # replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

        # now get the number of input features for the mask classifier
        in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
        hidden_layer = 256
        # and replace the mask predictor with a new one
        model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                           hidden_layer,
                                                           num_classes)

        return model

    def showbbox(self, model, img):
        # 輸入的img是0-1范圍的tensor
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        transforms = T.Compose([T.ToTensor()])
        img = transforms(img)
        model.eval()
        with torch.no_grad():
            '''
            prediction形如：
            [{'boxes': tensor([[1492.6672,  238.4670, 1765.5385,  315.0320],
            [ 887.1390,  256.8106, 1154.6687,  330.2953]], device='cuda:0'), 
            'labels': tensor([1, 1], device='cuda:0'), 
            'scores': tensor([1.0000, 1.0000], device='cuda:0')}]
            '''
            prediction = model([img.to(device)])

        img = img.permute(1, 2, 0)  # C,H,W → H,W,C，用來畫圖
        img = (img * 255).byte().data.cpu()  # * 255，float轉0-255
        img = np.array(img)  # tensor → ndarray
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        mask = np.array(prediction[0]['masks'].detach().cpu() * 255)
        mask = mask.astype("uint8")

        for i in range(prediction[0]['boxes'].cpu().shape[0]):
            if prediction[0]['scores'][i] < 0.5:
                continue
            xmin = round(prediction[0]['boxes'][i][0].item())
            ymin = round(prediction[0]['boxes'][i][1].item())
            xmax = round(prediction[0]['boxes'][i][2].item())
            ymax = round(prediction[0]['boxes'][i][3].item())
            label = prediction[0]['labels'][i].item()
            mm = mask[i][0]
            contours, hierarchy = cv2.findContours(mm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            scores = prediction[0]['scores'][i]
            # Draw contours:
            if label == 1:
                # cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 4)
                cv2.putText(img, f'{scores:.3f}', (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 0), 5)
            elif label == 2:
                # cv2.drawContours(img, contours, -1, (0, 0, 255), 1)
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 4)
                cv2.putText(img, 'mark_type_2', (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0))

        return img

    def run_model_method(self, img):
        """
        img: BGR image from opencv or numpy array
        """
        img_cvt = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im_pil = Image.fromarray(img_cvt)

        self.model.eval()
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        self.model.to(device)
        img = self.showbbox(self.model, im_pil)
        return img


if __name__ == "__main__":
    loadai = LoadAIModel("31_tool_knife.pth")
    num_class = 2
    # model = get_model_instance_segmentation(num_class)
    # model.load_state_dict(torch.load())
    # model.eval()
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    # loadai.run_model_method("img")

    # model.to(device)

    # for i in range(len(dataset_test)):
    #     img, _ = dataset_test[i]
    #     showbbox(model, img)

