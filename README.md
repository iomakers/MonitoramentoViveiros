# Monitoramento Viveiros


# Problemática

A criação produtiva de camarão depende muito da qualidade da água, sendo necessário monitorar os dados de temperatura e ph da água para otimizar a produção. Esse monitoramento pode ser realizado através de um sistema Scada que usará essa informação para criar gráficos e dar alertas da situação do viveiro. Um exemplo de atuação do sistema será em situações de chuva e que o sistema pode atuar automaticamente para minimizar o efeito da chuva na qualidade da água. Outro exemplo, será na situação de troca de água, o sistema irá monitorar as águas nos dois tanques e só misturar a água quando elas estiverem na mesma temperatura




# Objetivos

Criar sistema de monitoramento de viveiros

# Ferramentas

* Furadeira
* Alicate crimpar


# Materiais

* Esp32 dev
* DHT11
* Sensor de chuva
* 2 Relés
* 2 Contatoras
* Sensor Distância HC-SR04
* Placas PCB
* Suportes trilho din (link)[https://pt.aliexpress.com/item/Suporte-da-Placa-de-Circuito-PCB-35mm-Montagem-em-Trilho-DIN-Adaptador-Titular-Transportadora-Clips-L15/32834563788.html?src=google&albslr=230440671&isdl=y&aff_short_key=UneMJZVf&source=%7Bifdyn%3Adyn%7D%7Bifpla%3Apla%7D%7Bifdbm%3ADBM&albch=DID%7D&acnt=494-037-6276&albcp=757120293&albag=45449013012&slnk=&trgt=61865531738&plac=&crea=pt32834563788&netw=g&device=c&mtctp=&gclid=EAIaIQobChMIy6T2q5OC2AIVSgaRCh3l2wylEAQYASABEgLO_PD_BwE]
* Trilho Din
* Quadro Elétrico
* Bomba de água
* Eletrodos de PH
* Raspberry 3

# Software

* Arduino IDE
* ScadaBR

# Funcionalidades

O sistema coletará os dados de ph e temperatura da água, a humidade e temperatura do ambiente, sistema de alerta para chuva, sistema de bombeamento para tanque. Esse temporizador será implementado no ScadaBR.


# Etapas Desenvolvimento

- [ ] Ligação DHT11 ao ESP32
- [ ] Leitura DHT11 ao ESP32
- [ ] Ligação de relé ao ESP32
- [ ] Controle de bomba de água via relé
- [ ] Ligação sensor distancia ao ESP32
- [ ] Leitura sensor de humidade ao ESP32
- [ ] Instalação ScadaBR no Raspberry 3
- [ ] Desenvolvimento de supervisório no Scada BR
- [ ] Transmissão de dados aos Raspberry usando a biblioteca modbus ip
- [ ] Implementação e teste de temporizador no Raspberry
- [ ] Testes em campo
