"""The segmenter is largely based on the original implementation from Syndrone.
https://github.com/LTTM/Syndrone"""

from torchvision.models.segmentation import deeplabv3_resnet50
import torch
from torch import nn
import numpy as np
import os
from typing import Dict, Union, Tuple

from managed_subsystem.utils.metrics import Metrics


class LateFuse(nn.Module):
    def __init__(
        self, num_classes: int, rgb_model_path: str = "", dth_model_path: str = ""
    ) -> None:
        """
        Initializes the LateFuse model.

        Parameters:
        num_classes (int): The number of output classes for segmentation.
        rgb_model_path (str): Path to the pre-trained model for RGB input.
        dth_model_path (str): Path to the pre-trained model for depth input.
        """
        super().__init__()

        self.rgb = deeplabv3_resnet50(num_classes=num_classes, weights=None, weights_backbone=None)
        self.dth = deeplabv3_resnet50(num_classes=num_classes, weights=None, weights_backbone=None)
        self.merge = nn.Sequential(
            nn.Conv2d(2 * num_classes, 2 * num_classes, 1),
            nn.ReLU(),
            nn.Conv2d(2 * num_classes, 2 * num_classes, 1),
            nn.ReLU(),
            nn.Conv2d(2 * num_classes, num_classes, 1),
        )

        # load pre-trained model
        if rgb_model_path != "":
            self.rgb.load_state_dict(torch.load(rgb_model_path, map_location="cpu"))
        if dth_model_path != "":
            self.dth.load_state_dict(torch.load(dth_model_path, map_location="cpu"))

    def forward(self, c: torch.Tensor, d: torch.Tensor) -> Dict[str, torch.Tensor]:
        """
        Forward pass through the model.

        Parameters:
        c (torch.Tensor): RGB input tensor.
        d (torch.Tensor): Depth input tensor.

        Returns:
        Dict[str, torch.Tensor]: Output tensor with segmentation results.
        """
        c = self.rgb(c)["out"]
        d = self.dth(d)["out"]
        x = torch.cat([c, d], dim=1)
        x = self.merge(x)
        return {"out": x}


class UAVidSegmenter:
    def __init__(
        self, checkpoints_path: str = "checkpoints", modality: str = "fusion", logger=None
    ) -> None:
        """
        Initializes the UAVidSegmenter model.

        Parameters:
        checkpoints_path (str): Path to the directory containing model checkpoints.
        modality (str): Modality for segmentation. Options are 'fusion', 'rgb', and 'depth'.
        logger: Logger object for logging messages. Can be a ROS2 logger or None.
        """
        self.models = {
            "fusion": LateFuse(28),
            "rgb": deeplabv3_resnet50(num_classes=28, weights=None, weights_backbone=None),
            "depth": deeplabv3_resnet50(num_classes=28, weights=None, weights_backbone=None),
        }
        self.device = "cuda:1" if torch.cuda.is_available() else "cpu"
        #self.device = "cpu" 

        try:
            logger.info(f"Running on device {self.device}")
        except:
            pass

        for model_type in self.models:
            weights_path = os.path.join(checkpoints_path, model_type + ".pth")
            if os.path.exists(weights_path):
                self.models[model_type].load_state_dict(
                    torch.load(weights_path, map_location="cpu")
                )
                self.models[model_type].to(self.device)
                self.models[model_type].eval()
            else:
                log_string = (
                    f"Weights path {weights_path} does not exist. Loading random weights instead."
                )
                try:
                    logger.warn(log_string)
                except:
                    print(log_string)

        self.modality = modality

        self.label_names = [
            "Building",
            "Fence",
            "Other",
            "Pole",
            "RoadLine",
            "Road",
            "Sidewalk",
            "Vegetation",
            "Wall",
            "Traffic Signs",
            "Sky",
            "Ground",
            "Bridge",
            "Rail Track",
            "Guard Rail",
            "Traffic Light",
            "Static",
            "Dynamic",
            "Water",
            "Terrain",
            "Person",
            "Rider",
            "Car",
            "Truck",
            "Bus",
            "Train",
            "Motorcycle",
            "Bicycle",
        ]
        # ID mapping stems from the original SynDrone implementation to map CARLA ids to 28 relevant classes
        self.idmap = {
            1: 0,
            2: 1,
            3: 2,
            5: 3,
            6: 4,
            7: 5,
            8: 6,
            9: 7,
            11: 8,
            12: 9,
            13: 10,
            14: 11,
            15: 12,
            16: 13,
            17: 14,
            18: 15,
            19: 16,
            20: 17,
            21: 18,
            22: 19,
            40: 20,
            41: 21,
            100: 22,
            101: 23,
            102: 24,
            103: 25,
            104: 26,
            105: 27,
        }
        self.cmap = np.array(
            [
                [70, 70, 70],  # building
                [190, 153, 153],  # fence
                [180, 220, 135],  # other
                [153, 153, 153],  # pole
                [255, 255, 255],  # road line
                [128, 64, 128],  # road
                [244, 35, 232],  # sidewalk
                [107, 142, 35],  # vegetation
                [102, 102, 156],  # wall
                [220, 220, 0],  # traffic sign
                [70, 130, 180],  # sky
                [81, 0, 81],  # ground
                [150, 100, 100],  # bridge
                [230, 150, 140],  # rail track
                [180, 165, 180],  # guard rail
                [250, 170, 30],  # traffic light
                [110, 190, 160],  # static
                [111, 74, 0],  # dynamic
                [45, 60, 150],  # water
                [152, 251, 152],  # terrain
                [220, 20, 60],  # person
                [255, 0, 0],  # rider
                [0, 0, 142],  # car
                [0, 0, 70],  # truck
                [0, 60, 100],  # bus
                [0, 80, 100],  # train
                [0, 0, 230],  # motorcycle
                [119, 11, 32],  # bicycle
                [0, 0, 0],  # unknown
            ],
            dtype=np.uint8,
        )

    def prepare_rgb(self, rgb: np.ndarray) -> Union[torch.Tensor, None]:
        """
        Prepares RGB input for the model.

        Parameters:
        rgb (np.ndarray): RGB image as a numpy array.

        Returns:
        Union[torch.Tensor, None]: Preprocessed RGB tensor or None if input is invalid.
        """
        if not isinstance(rgb, np.ndarray):
            return None
        rgb = rgb[..., ::-1] / 255.0
        rgb = (rgb - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        return torch.from_numpy(rgb).permute(2, 0, 1).float().unsqueeze(0).to(self.device)

    def prepare_dth(self, dth: np.ndarray) -> Union[torch.Tensor, None]:
        """
        Prepares depth input for the model.

        Parameters:
        dth (np.ndarray): Depth image as a numpy array.

        Returns:
        Union[torch.Tensor, None]: Preprocessed depth tensor or None if input is invalid.
        """
        if not isinstance(dth, np.ndarray):
            return None
        dth = dth.astype(np.float32) / (256 * 256 - 1)
        dth = np.expand_dims(dth, 0)
        dth = np.concatenate((dth, dth, dth), axis=0)
        return torch.from_numpy(dth).float().unsqueeze(0).to(self.device)

    def prepare_mlb(self, sem: np.ndarray) -> torch.Tensor:
        """
        Prepares label with ids from 0...28 from semantic CARLA labels.

        Parameters:
        sem (np.ndarray): Semantic label image as a numpy array.

        Returns:
        torch.Tensor: Label map tensor on the device.
        """
        mlb = -1 * np.ones_like(sem, dtype=int)
        for k, v in self.idmap.items():
            mlb[sem == k] = v
        return torch.from_numpy(mlb).long().to(self.device)

    def calc_metrics(self, pred: torch.Tensor, mlb: torch.Tensor) -> dict:
        """
        Calculates the mean Intersection over Union (mIoU) metric.

        Parameters:
        pred (torch.Tensor): Predicted segmentation tensor.
        mlb (torch.Tensor): Ground truth label map tensor.

        Returns:
        float: Percent mIoU metric value.
        """
        metrics = Metrics(self.label_names, device=self.device)
        metrics.add_sample(pred.to(self.device), mlb.to(self.device))
        iou_dict = {key: value for key, value in zip(self.label_names, metrics.IoU().detach().cpu().numpy())}
        return iou_dict

    def color_label(self, ts: np.ndarray) -> np.ndarray:
        """
        Maps label IDs to colors using a predefined color map.

        Parameters:
        ts (np.ndarray): Input label array.

        Returns:
        np.ndarray: Color-mapped label image.
        """
        return self.cmap[np.array(ts)]

    def to_rgb(self, ts: np.ndarray) -> np.ndarray:
        """
        Converts tensor image to RGB format.

        Parameters:
        ts (np.ndarray): Input tensor image.

        Returns:
        np.ndarray: RGB converted image.
        """
        return np.array(ts) * [0.229, 0.224, 0.225] + [0.485, 0.456, 0.406]

    def compute_entropy_4D(self, tensor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the entropy on a 4D tensor with shape (number_of_classes, 256, 256, 256).
    
        Parameters:
            tensor (np.ndarray): 4D tensor of shape (number_of_classes, 256, 256, 256)

        Returns:
            np.ndarray: 3D tensor of shape (256, 256, 256) with entropy values for each pixel.
        """
    
        # First, normalize the tensor along the class axis so that it represents probabilities
        sum_tensor = np.sum(tensor, axis=1, keepdims=True)
        tensor_normalized = tensor / sum_tensor

        # Calculate entropy
        entropy_elements = -tensor_normalized * np.log2(np.clip(tensor_normalized, 1e-10, 1.0))  # Added a small value to avoid log(0)
        entropy = np.sum(entropy_elements, axis=0)

        #entropy = np.transpose(entropy, (2,1,0))

        avg_entropy = np.average(entropy)

        return entropy, avg_entropy
        

    def inference(self, data: Dict[str, np.ndarray], modality: str = "") -> Tuple[np.ndarray, float]:
        """
        Performs segmentation inference.

        Parameters:
        data (Dict[str, np.ndarray]): Dictionary containing 'rgb' and 'depth' images.
        modality (str): Modality for inference. Options are 'fusion', 'rgb', and 'depth'.

        Returns:
        np.ndarray: Predicted segmentation map as a numpy array.
        float: entropy
        """
        if modality != "":
            self.modality = modality

        assert modality in ["fusion", "rgb", "depth"]

        if self.modality == "fusion":
            rgb = self.prepare_rgb(data["rgb"])
            dth = self.prepare_dth(data["depth"])
            out = self.models[self.modality](rgb, dth)
        if self.modality == "rgb":
            rgb = self.prepare_rgb(data["rgb"])
            out = self.models[self.modality](rgb)
        if self.modality == "depth":
            dth = self.prepare_dth(data["depth"])
            out = self.models[self.modality](dth)
        _, entropy = self.compute_entropy_4D(out["out"].cpu().detach().numpy())
        return out["out"].argmax(dim=1)[0].cpu().numpy().astype(np.uint8), entropy
