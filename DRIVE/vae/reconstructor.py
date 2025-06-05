import os
import glob
import torch
import matplotlib.pyplot as plt
from torch.nn import functional as F
from torch.utils.data import DataLoader
import torchvision.transforms as transforms

from models import VAE, AE
from loaders import ImagePairDataset, DatasetFromSubset

# Hyper-parameters
BATCH_SIZE = 1
LATENT_SPACE = 64

train_type = 'rgb/' # 'segmentation/'
label_type = 'segmentation/'
train_data_dir = 'data/' + train_type
label_data_dir = 'data/' + label_type
save_dir = 'models/my_model_vae/'
fig_save_dir = 'models/for_samples/'

if not os.path.exists(fig_save_dir):
    os.makedirs(fig_save_dir)

def loss_function(recon_x, x):
    BCE = F.mse_loss(recon_x, x, size_average=False)
    return BCE

def main():
    train_transforms = transforms.Compose([
        transforms.ToPILImage(),
        transforms.ColorJitter(brightness=0.3, contrast=0.2, saturation=0.2, hue=0.05),
        transforms.RandomAffine(degrees=0, scale=(1, 1.2)),
        transforms.RandomGrayscale(0.05),
        transforms.ToTensor(),  # convert the PIL Image to a PyTorch tensor
        transforms.RandomErasing(p=0.1, scale=(0.02, 0.05), value=0)
        # p is the probability of applying the transformation
    ])

    dataset = ImagePairDataset(train_data_dir, label_data_dir)
    train_set = DatasetFromSubset(dataset, train_transforms)
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=BATCH_SIZE)

    model = VAE(latent_size=LATENT_SPACE)

    model.load(save_dir + str(129) + '.pth')
    i = -1
    with torch.no_grad(): # No need to track the gradients
        for x, y in train_loader:
            i += 1
            if i % 100 != 0:
                continue
            # Decode data
            recon_batch, mu, logvar = model(x)

            fig = plt.figure()
            plt.subplot(3, 1, 1)
            a = recon_batch.squeeze().numpy()
            plt.imshow(a, cmap='gray')
            plt.subplot(3, 1, 2)
            b = y.squeeze()[0].numpy()
            plt.imshow(b, cmap='gray')
            plt.subplot(3, 1, 3)
            c = x.squeeze().permute(1, 2, 0).numpy()
            plt.imshow(c)
            plt.savefig(fig_save_dir + str(i) + '.png')
            plt.close()


    # model_list = sorted(glob.glob(save_dir + '*.pth'))
    # for i in range(len(model_list)):
    #     if i < 50:
    #         continue

    # for i in [59, 69, 93, 99, 105, 106, 112, 117, 118]:
    #     model.load(save_dir + str(i) + '.pth')
    #     total_loss = 0
    #     j = 0
    #     with torch.no_grad(): # No need to track the gradients
    #         for x, y in train_loader:
    #             if j % 100 != 0:
    #                 continue
    #
    #             # Decode data
    #             recon_batch = model(x)
    #
    #             a = recon_batch.squeeze().numpy()
    #             b = (y * 255 / 12).squeeze()[0].numpy()
    #
    #             loss = loss_function(torch.Tensor(a), torch.Tensor(b))
    #             total_loss += loss.item()
    #             j += 1
    #         print('model {}, loss {}'.format(i, total_loss))

if __name__ == "__main__":
    main()

