import cv2 as cv
import numpy as np
from skimage.filters import frangi

def vessel_segmentation(input_image):
    # Abre la imagen y pásala a LAB
    img = cv.imread(input_image)
    lab_image = cv.cvtColor(img, cv.COLOR_BGR2LAB)
    
    # Extrae el canal L
    l_channel, a_channel, b_channel = cv.split(lab_image)
    
    # Haz CLAHE al canal L
    clahe = cv.createCLAHE(clipLimit=20.0, tileGridSize=(8, 8))
    l_channel_clahe = clahe.apply(l_channel)
    
    # Une de nuevo todos los canales y convierte a RGB
    lab_image_clahe = cv.merge([l_channel_clahe, a_channel, b_channel])
    rgb_image = cv.cvtColor(lab_image_clahe, cv.COLOR_LAB2BGR)
    
    # Extrae el canal verde para el filtro de Frangi
    green_channel = rgb_image[:, :, 1]
    
    # Aplica el filtro de Frangi
    filtered_image = frangi(green_channel)
    
    # Gaussian blur
    filtered_image = cv.GaussianBlur(filtered_image, (3, 3), 0)
    
    
    # Segmentación por umbralización
    filtered_image = filtered_image > 0.05
    
    # Aplica un cierre para eliminar huecos
    kernel = np.ones((3, 3), np.uint8)
    
    filtered_image = cv.morphologyEx(filtered_image.astype(np.uint8), cv.MORPH_CLOSE, kernel)
    
    
        # Segmentación por etiquetado de componentes conectadas
    num_labels, labels_im = cv.connectedComponents(filtered_image.astype(np.uint8))
    labels_im = labels_im.astype(np.uint8)
    labels_im = labels_im * (255)
    labels_im = labels_im.astype(np.uint8)
    
    
    return labels_im

# Apply the vessel segmentation function to the uploaded image
input_image_path = 'TVAProject/input/21.png'
segmented_image_result = vessel_segmentation(input_image_path)

# Plot the results
plt.imshow(segmented_image_result, cmap='gray')
#dont show the axis
plt.axis('off')
plt.show()


# Plot the results
img = cv.imread('TVAProject/gt/21.png', cv.IMREAD_GRAYSCALE)
plt.imshow(img, cmap='gray')
#dont show the axis
plt.axis('off')
plt.show()

# Save the results
output_image_path = 'save.png'
cv.imwrite(output_image_path, segmented_image_result)


Mean IoU=0.343182189939336