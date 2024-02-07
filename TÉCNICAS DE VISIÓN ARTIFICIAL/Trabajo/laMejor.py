def preprocess(image):
    # Convertir a modalidad LAB
    lab = cv.cvtColor(image, cv.COLOR_BGR2LAB)
    l, a, b = cv.split(lab)

    # Se aplica Ecualización Adaptativa de Histograma con Limitación de Contraste
    clahe = cv.createCLAHE(clipLimit=3.0)
    cl = clahe.apply(l)
    limg = cv.merge((cl, a, b))

    # Convertir de nuevo la modalidad LAB a RGB
    final = cv.cvtColor(limg, cv.COLOR_LAB2BGR)
            
    # Extraer canal verde
    _, green, _ = cv.split(final)
        
    windows = [(3,3), (5,5), (7,7), (9,9), (11,11)]
        
    morphed = green.copy()
        
    # Aplicar operaciones morfológicas con diferentes tamaños de ventana
    for window in windows:
        morphed = cv.morphologyEx(morphed, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_ELLIPSE, window), iterations = 1)
        morphed = cv.morphologyEx(morphed, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, window), iterations = 1)
        
    # Restar la imagen original del resultado de las operaciones morfológicas
    imgSubstracted = cv.subtract(morphed, green)
    imgSubstracted = clahe.apply(imgSubstracted)
        
    return imgSubstracted

def features(image):

    #removing very small contours through area parameter noise removal
    ret, f6 = cv.threshold(image, 15, 255, cv.THRESH_BINARY)

    mask = np.ones(image.shape[:2], dtype='uint8') * 255
    contours, hierarchy = cv.findContours(f6.copy(),cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv.contourArea(cnt) <= 255:
            cv.drawContours(mask, [cnt], -1, 0, -1)
            
    im = cv.bitwise_and(image, image, mask=mask)
    _ , fin = cv.threshold(im, 15, 255, cv.THRESH_BINARY_INV)
    newfin = cv.erode(fin, cv.getStructuringElement(cv.MORPH_ELLIPSE, (3,3)), iterations=1)
        

    #removing blobs of unwanted size
    fundus_eroded = cv.bitwise_not(newfin)
    xmask = np.ones(fundus_eroded.shape[:2], dtype='uint8') * 255
    xcontours, xhierarchy = cv.findContours(fundus_eroded.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    
    
    for cnt in xcontours:
        shape = 'unidentified'
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.04 * peri, False)
        if len(approx) > 4 and cv.contourArea(cnt) <= 3000 and cv.contourArea(cnt) >= 100:
            shape = 'circle'
        else:
            shape = 'vessels'
            
        if(shape == 'circle'):
            cv.drawContours(xmask, [cnt], -1, 0, -1)

    finimage = cv.bitwise_and(fundus_eroded, fundus_eroded, mask=xmask)
    blood_vessels = cv.bitwise_not(finimage)
    kernel = np.ones((2,2), np.uint8)
    blood_vessels = cv.subtract(255, blood_vessels)
        

    new1 = blood_vessels
                 
    # Create a mask of zeros with the same size as the original image
    mask = np.zeros_like(new1)

    # Get the center of the image
    height, width = new1.shape[:2]
    center = (width//2, height//2)
        
    # Draw the circle on the mask
    cv.circle(mask, center, 255, (255,255,255), -1)

    # Subtract the mask from the original image
    result = cv.bitwise_and(new1, mask)
    
    # Save the image
    cv.imwrite('bloodVessels.jpg', result)
    
    # recorta los objetos pequeños
    kernel = np.ones((2,2), np.uint8)
    result = cv.erode(result, kernel, iterations=1)
        
    return result
    
def vessel_segmentation(input_image):
    # Step1: Read the retinal image
    retinal_image = cv.imread(input_image, cv.IMREAD_COLOR)
    # Step2: Abstract the green channel of the input image
    green = preprocess(retinal_image)
    img = features(green)
    return img


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