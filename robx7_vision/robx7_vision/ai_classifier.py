"""Clasificador de recortes con una red neuronal pequena escrita en NumPy.

Archivo sin ROS. Lo usa robx7_vision/object_detector.py cuando classifier vale ai.
Los datos de entrenamiento los genera robx7_vision/synthetic_data.py.
El entrenamiento se lanza con tools/train_classifier.py.

Que hace
  Recibe el recorte de un objeto y responde su color y su forma, por ejemplo red_cube.

Como lo hace
  1. El recorte se reduce a 16 por 16 pixeles y se aplana en un vector de 768 numeros.
  2. Una red de dos capas lo transforma, 768 entradas, 32 neuronas ocultas, 6 salidas.
  3. Softmax convierte las 6 salidas en probabilidades. Gana la mayor.
"""
import cv2
import numpy as np

CLASSES = ['red_cube', 'red_cylinder', 'green_cube', 'green_cylinder',
           'blue_cube', 'blue_cylinder']
IMG_SIZE = 16


def crop_contour(frame_bgr, contour, margin=4):
    """Recorta el rectangulo que rodea al contorno, con un margen de fondo."""
    x, y, w, h = cv2.boundingRect(contour)
    h_img, w_img = frame_bgr.shape[:2]
    x0, y0 = max(0, x - margin), max(0, y - margin)
    x1, y1 = min(w_img, x + w + margin), min(h_img, y + h + margin)
    return frame_bgr[y0:y1, x0:x1]


def extract_features(crop_bgr):
    """Reduce el recorte a 16 por 16 y lo convierte en un vector con valores entre 0 y 1."""
    small = cv2.resize(crop_bgr, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_AREA)
    return small.astype(np.float32).reshape(-1) / 255.0


class MLP:
    """Perceptron multicapa con una capa oculta ReLU y salida softmax."""

    def __init__(self, n_in, n_hidden, n_out, seed=0):
        rng = np.random.default_rng(seed)
        # Inicializacion de He. Mantiene la escala de las senales al atravesar la capa
        self.w1 = rng.normal(0, np.sqrt(2.0 / n_in), (n_in, n_hidden)).astype(np.float32)
        self.b1 = np.zeros(n_hidden, np.float32)
        self.w2 = rng.normal(0, np.sqrt(2.0 / n_hidden), (n_hidden, n_out)).astype(np.float32)
        self.b2 = np.zeros(n_out, np.float32)

    def forward(self, x):
        hidden = np.maximum(0.0, x @ self.w1 + self.b1)    # ReLU
        logits = hidden @ self.w2 + self.b2
        return hidden, logits

    @staticmethod
    def softmax(logits):
        z = logits - logits.max(axis=-1, keepdims=True)     # resta el maximo por estabilidad
        e = np.exp(z)
        return e / e.sum(axis=-1, keepdims=True)

    def predict_proba(self, x):
        return self.softmax(self.forward(x)[1])

    def fit(self, x, y, epochs=40, lr=0.003, batch=64, l2=1e-4, seed=0, on_epoch=None):
        """Entrena con descenso de gradiente Adam sobre la perdida de entropia cruzada."""
        rng = np.random.default_rng(seed)
        params = [self.w1, self.b1, self.w2, self.b2]
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        step = 0
        n = len(x)
        for epoch in range(epochs):
            order = rng.permutation(n)
            total_loss = 0.0
            for start in range(0, n, batch):
                idx = order[start:start + batch]
                xb, yb = x[idx], y[idx]
                hidden, logits = self.forward(xb)
                probs = self.softmax(logits)
                total_loss += -np.log(probs[np.arange(len(yb)), yb] + 1e-9).sum()

                # Retropropagacion. El gradiente de softmax con entropia cruzada es probs menos uno
                d_logits = probs
                d_logits[np.arange(len(yb)), yb] -= 1.0
                d_logits /= len(yb)
                g_w2 = hidden.T @ d_logits + l2 * self.w2
                g_b2 = d_logits.sum(axis=0)
                d_hidden = (d_logits @ self.w2.T) * (hidden > 0)
                g_w1 = xb.T @ d_hidden + l2 * self.w1
                g_b1 = d_hidden.sum(axis=0)

                step += 1
                for i, (p, g) in enumerate(zip(params, [g_w1, g_b1, g_w2, g_b2])):
                    m[i] = 0.9 * m[i] + 0.1 * g
                    v[i] = 0.999 * v[i] + 0.001 * g * g
                    m_hat = m[i] / (1 - 0.9 ** step)
                    v_hat = v[i] / (1 - 0.999 ** step)
                    p -= lr * m_hat / (np.sqrt(v_hat) + 1e-8)
            if on_epoch is not None:
                on_epoch(epoch + 1, total_loss / n)

    def save(self, path):
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        net = cls(data['w1'].shape[0], data['w1'].shape[1], data['w2'].shape[1])
        net.w1, net.b1, net.w2, net.b2 = data['w1'], data['b1'], data['w2'], data['b2']
        return net


class CropClassifier:
    """Envuelve la red para usarla directamente con recortes de imagen."""

    def __init__(self, net, classes=CLASSES):
        self.net = net
        self.classes = list(classes)

    @classmethod
    def load(cls, path):
        return cls(MLP.load(path))

    def predict(self, crop_bgr):
        """Devuelve color, forma y probabilidad de la clase ganadora."""
        probs = self.net.predict_proba(extract_features(crop_bgr)[None, :])[0]
        best = int(np.argmax(probs))
        color, shape = self.classes[best].split('_')
        return color, shape, float(probs[best])
