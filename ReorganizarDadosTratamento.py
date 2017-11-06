wq=open('/home/paulo/ProjetoKinectLRVA/FramesTratados/DadosTratados.txt', 'w')
cont = 1
while (1):

	if (cont==64):
		while(cont<80):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1
	if (cont==116):
		while(cont<117):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1
	if (cont==238):
		while(cont<239):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1
	if (cont==241):
		while(cont<242):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1		
	if (cont==269):
		while(cont<272):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1
	if (cont==286):
		while(cont<292):
			texto = texto	
			wq.write(texto)
			wq.write('\n')
			cont+=1
	try:
		DadosProfundidade='/home/paulo/ProjetoKinectLRVA/FramesTratados/POSFrameTratado'
		DadosProfundidade=DadosProfundidade+str(cont)
		rq = open(DadosProfundidade, 'r')
		texto = rq.read()
	except ValueError:
		cont+=1
	texto = texto	
	wq.write(texto)
	wq.write('\n')
	print(cont)
	cont+=1
rq.close()
#wq.close()

