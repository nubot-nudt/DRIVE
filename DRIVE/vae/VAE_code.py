import os
import torch
from tqdm import tqdm
from torch.nn import functional as F
from tensorboardX import SummaryWriter
from torchvision.utils import save_image
from torch.utils.data import random_split
import torchvision.transforms as transforms

from models import VAE
from misc import RandomMotionBlur
from learning import EarlyStopping, ReduceLROnPlateau
from loaders import ImagePairDataset, DatasetFromSubset

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torch")

torch.backends.cudnn.benchmark = True

# Hyper-parameters
samples = True
NUM_EPOCHS = 1000
BATCH_SIZE = 32
LEARNING_RATE = 1e-3
LATENT_SPACE = 64

train_type = 'rgb/' # 'segmentation/'
label_type = 'segmentation/'
train_data_dir = 'data/' + train_type
label_data_dir = 'data/' + label_type
save_dir = 'models/my_model/'
log_dir = 'models/logs/'
sample_dir = 'models/samples/'

if not os.path.exists(save_dir):
    os.makedirs(save_dir)
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
if not os.path.exists(sample_dir):
    os.makedirs(sample_dir)

writer = SummaryWriter(log_dir)

def loss_function(recon_x, x, mu, logsigma):
    """ VAE loss function """
    BCE = F.mse_loss(recon_x, x, size_average=False)

    # https://arxiv.org/abs/1312.6114
    # 0.5 * sum(1 + log(sigma^2) - mu^2 - sigma^2)
    KLD = -0.5 * torch.sum(1 + 2 * logsigma - mu.pow(2) - (2 * logsigma).exp())
    return BCE + KLD

def train(model, train_loader, optim, epoch):
    model.train()
    train_loss = 0
    for input_image, output_image in tqdm(train_loader, desc="Train Set"):
        recon_batch, mu, logvar = model(input_image)
        loss = loss_function(recon_batch[:, 0, :, :], (output_image * 255 / 12)[:, 0, :, :], mu, logvar)
        train_loss += loss.item()

        optim.zero_grad()
        loss.backward()
        optim.step()

    if samples:
        save_image(input_image[0], os.path.join(sample_dir + 'gt_' + str(epoch) + '.png'))
        save_image(recon_batch[0], os.path.join(sample_dir + str(epoch) + '.png'))

    return train_loss / len(train_loader)

def test(model, test_loader):
    model.eval()
    test_loss = 0
    with torch.no_grad():
        for input_image, output_image in tqdm(test_loader, desc="Test Set"):
            recon_batch, mu, logvar = model(input_image)
            test_loss += loss_function(recon_batch[:, 0, :, :], (output_image * 255 / 12)[:, 0, :, :], mu, logvar).item()
    return test_loss / len(test_loader)

def main():
    train_transforms = transforms.Compose([
        transforms.ToPILImage(),
        transforms.ColorJitter(brightness=0.3, contrast=0.2, saturation=0.2, hue=0.05),
        transforms.RandomAffine(degrees=0, scale=(1, 1.2)),
        transforms.RandomGrayscale(0.05),
        RandomMotionBlur(p=0.2, kernel_size=5, angle_range=(-20, 20)),
        # transforms.RandomHorizontalFlip(p=0.5),
        transforms.ToTensor(),  # convert the PIL Image to a PyTorch tensor
        transforms.RandomErasing(p=0.1, scale=(0.02, 0.05), value=0)  # p is the probability of applying the transformation
        ])

    dataset = ImagePairDataset(train_data_dir, label_data_dir)
    train_size = int(len(dataset) * 0.8)
    val_size = int(len(dataset) * 0.2)
    train_set, val_set = random_split(dataset, [train_size, val_size])
   
    train_set = DatasetFromSubset(train_set, train_transforms)
    val_set = DatasetFromSubset(val_set)

    train_loader = torch.utils.data.DataLoader(train_set, batch_size=BATCH_SIZE)
    valid_loader = torch.utils.data.DataLoader(val_set, batch_size=BATCH_SIZE)

    model = VAE(latent_size=LATENT_SPACE)
    optim = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = ReduceLROnPlateau(optim, 'min', factor=0.5, patience=10, verbose=True)
    earlystopping = EarlyStopping('min', patience=25)

    for epoch in range(NUM_EPOCHS):
        train_loss = train(model, train_loader, optim, epoch)
        val_loss = test(model, valid_loader)

        scheduler.step(val_loss)
        earlystopping.step(val_loss)

        writer.add_scalar('Loss/Train Loss', train_loss, global_step=epoch)
        writer.add_scalar('Loss/Valid Loss', val_loss, global_step=epoch)
        print('EPOCH {}/{} \t train loss {:.3f} \t val loss {:.3f}'.format(epoch + 1, NUM_EPOCHS,train_loss,val_loss))
        model.save(save_dir + str(epoch) + '.pth')

if __name__ == "__main__":
    main()
