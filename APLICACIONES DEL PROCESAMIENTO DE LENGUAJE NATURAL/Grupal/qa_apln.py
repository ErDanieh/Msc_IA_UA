import faiss
from faiss import write_index
import numpy as np
import pandas as pd
import torch
from transformers import AutoTokenizer, AutoModel
from datasets import load_dataset


############################################################################################################
#  Recuperación de información
############################################################################################################

# Función para obtener word embeddings
def get_embeddings(documents):
  tokenizer = AutoTokenizer.from_pretrained('bert-base-uncased')
  model = AutoModel.from_pretrained('bert-base-uncased').to(device)
  embeddings = []
  for i, doc in enumerate(documents):
    print(f"Documento {i}")
    inputs = tokenizer(doc, return_tensors='pt', truncation=True, padding=True, max_length=512).to(device)
    with torch.no_grad():
      outputs = model(**inputs)
    doc_embedding = outputs.last_hidden_state.mean(1).squeeze().cpu().numpy()
    embeddings.append(doc_embedding)
  return np.array(embeddings)

# Función para crear el índice FAISS
def create_faiss_index(embeddings):
  dimension = embeddings.shape[1]
  index = faiss.IndexFlatL2(dimension)
  index.add(embeddings)
  return index

# Función para obtener los embeddings de una consulta de texto
def get_query_embedding(query_text):
    tokenizer = AutoTokenizer.from_pretrained('bert-base-uncased')
    model = AutoModel.from_pretrained('bert-base-uncased').to(device)
    inputs = tokenizer(query_text, return_tensors='pt', truncation=True, padding=True, max_length=512).to(device)
    with torch.no_grad():
        outputs = model(**inputs)
    query_embedding = outputs.last_hidden_state.mean(1).squeeze().cpu().numpy()
    return query_embedding

def query_db(question):
    # Consulta
    query_embedding = get_query_embedding(question)
    query_vector = np.expand_dims(query_embedding, axis=0)

    # Realiza la búsqueda en el índice FAISS
    D, I = index.search(query_vector, k=5)  # Busca los 5 documentos más similares

    print("Consulta realizada con el texto:", question)
    print("Documentos más similares:")

    context = ""

    for i, idx in enumerate(I[0], start=1):
        # print(f"{i}: Documento {idx} con una distancia de {D[0][i-1]}")
        # print("Contenido del documento:", documents[idx])
        context += '\n' + documents[idx]

    return context

############################################################################################################
#  Preparar dataset
############################################################################################################

# Movemos el modelo y los datos al dispositivo que corresponda
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

squad_ds = load_dataset(path='squad_v2', name='squad_v2', split='train')

documents = list(set(squad_ds['context'])) # Eliminamos duplicados

index = faiss.read_index("squad.index")

############################################################################################################
#  Preparar fichero de referencias para evaluación
############################################################################################################

answers = []

for item in squad_ds['answers']:
    if len(item['text']) > 0:
        answers.append(item['text'][0])

answers = answers[:700] # Cogemos solo un subset de preguntas por ahorrar tiempo de ejecución

with open('ref.txt', 'w') as f:
    for answer in answers:
        f.write(answer + '\n')

############################################################################################################
#  Generación de texto
############################################################################################################
        
from transformers import pipeline

models = ['distilbert/distilbert-base-cased-distilled-squad','deepset/roberta-base-squad2','google-bert/bert-large-uncased-whole-word-masking-finetuned-squad','Intel/dynamic_tinybert']
questions = squad_ds['question']
questions = questions[:700]

contador = 0
for model in models:
    question_answerer = pipeline("question-answering", model=model)
    filename = 'hyp_' + str(contador) + '.txt' # Guardar las respuesta de cada modelo en un fichero diferente
    contador += 1
    with open(filename, 'w') as f:
        for i, question in enumerate(questions):
            print(f"Pregunta: {i}")
            context = query_db(question)

            model_answer = question_answerer(question=question, context=context)
            f.write(model_answer['answer'] + '\n')