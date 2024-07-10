---
sidebar_position: 4
custom_edit_url: null
title: "Análise Financeira da Prova de Conceito (PoC)"
---


# Análise Financeira da Prova de Conceito (PoC)

A Prova de Conceito (PoC) na análise financeira é uma abordagem utilizada para demonstrar a viabilidade prática de uma ideia ou projeto financeiro. Esta metodologia envolve a implementação inicial de um conceito ou estratégia financeira em uma escala limitada para testar sua eficácia e potencial de sucesso antes de uma implementação mais ampla.

A PoC é uma ferramenta valiosa para reduzir os riscos e incertezas associados a novas iniciativas financeiras, permitindo que as empresas coletem dados reais antes de realizar investimentos significativos.

:::danger
Esta POC foi realizada em `21/05/2024`. É importante ressaltar que, após essa data, os preços e os impostos aplicáveis podem ter sofrido alterações significativas. As condições de mercado estão em constante mudança, e fatores como variações cambiais, mudanças na legislação fiscal, e flutuações na oferta e demanda podem impactar os custos e impostos associados. 
:::

## Introdução

Esta análise financeira detalha os custos e considerações econômicas relacionados à Prova de Conceito (PoC) de um robô operado à distância, projetado para verificar a limpeza dos reboilers da empresa Atvos. A análise abrange custos diretos e indiretos, possíveis reduções de custos e estabelece uma base para determinar o preço mínimo aceitável para venda.

## Análise de Custos

Segue abaixo a análise de custos realizada, detalhando os diversos componentes e suas respectivas despesas. Para uma melhor compreensão e transparência dos dados apresentados, disponibilizamos também os arquivos Excel correspondentes. Estes arquivos incluem todas as informações necessárias para uma análise mais aprofundada e podem ser acessados através do link fornecido após as tabelas.

### Custos de Hardware

| Descrição                                      | Quantidade | Valor Unitário | Valor Final  | Fonte               |
| ---------------------------------------------- | ---------- | -------------- | ------------ | ------------------- |
| Bateria de Polímero de Lítio (Li-Po) de 1800mAh | 1          | R$ 160,00      | R$ 165,90    | Mercado Livre     |
| Bateria Reserva de Polímero de Lítio (Li-Po) de 2200mAh | 1          | R$ 178,00      | R$ 182,50    | Mercado Livre     |
| Câmera WebCam                                  | 1          | R$ 156,99      | R$ 156,99    | Amazon  |
| Robô Turtlebot 3 - Raspberry Pi 4 4GB          | 1          | R$ 6.593,94    | R$ 12.711,23 | AliExpress    |
| **Total**                                      |            |                | **R$ 13.216,62** |                   |

> Nota: A coluna “Valor Final” inclui valores de frete e impostos (considerando ICMS de 18% e IPI de 10%).

> Clique [aqui](https://1drv.ms/x/s!Arzvp0EDtndcgUcPq0qq6VL_te33?e=FgOShB) para abrir a planilha Excel

### Custos de Mão de Obra

| Função                | Quantidade | Meses | Salário Mensal | Valor Final    | Fonte                |
| --------------------- | ---------- | ----- | -------------- | -------------- | -------------------- |
| Engenheiro de Software| 2          | 2     | R$ 6.000,00    | R$ 24.000,00   | Glassdoor|
| Engenheiro Robótico   | 2          | 2     | R$ 7.000,00    | R$ 28.000,00   | Glassdoor|
| Gestor de Projetos    | 1          | 2     | R$ 7.500,00    | R$ 13.500,00   | Glassdoor|
| **Total**             |            |       |                | **R$ 65.500,00** |                    |

> Clique [aqui](https://1drv.ms/x/s!Arzvp0EDtndcgUl5MVMSg7RRGoXL?e=mLwYJj) para abrir a planilha Excel 

### Custos Totais

| Descrição             | Valor         |
| --------------------- | ------------- |
| Custos de Hardware    | R$ 13.216,62  |
| Custos de Mão de Obra | R$ 65.500,00  |
| **Total**             | **R$ 78.716,62** |

> Clique [aqui](https://1drv.ms/x/s!Arzvp0EDtndcgUuV7u5yx5Di6jn5?e=wWhbzP) para abrir a planilha Excel

### Impostos e CNAE

Para esta análise, consideramos um ICMS de 18% e um IPI de 10%, conforme aplicável para equipamentos eletrônicos e componentes de robótica. O CNAE considerado para este projeto é 6201-5/01 (Desenvolvimento de Programas de Computador Sob Encomenda), que pode influenciar as alíquotas de impostos aplicáveis.

## Otimização de Custos

Além de uma análise financeira, o grupo considerou onde e como pode fazer alterações para diminuir o valor da proposta, definindo claramente um valor mínimo aceitável para a venda do projeto. Abaixo, são apresentadas algumas estratégias para redução de custos e o cálculo do valor mínimo aceitável.

### Estratégias para Redução de Custos

- **Substituição de Componentes de Hardware**:
  - **Baterias**: Considerar baterias de fornecedores alternativos com preços mais competitivos.
  - **Câmera WebCam**: Optar por modelos com especificações similares, mas de marcas mais econômicas.
  - **Robô Turtlebot**: Avaliar alternativas de kits robóticos que ofereçam as funcionalidades necessárias a um custo menor, ou negociar descontos com o fornecedor atual.

- **Redução de Custos de Mão de Obra**:
  - **Engenheiros**: Contratar engenheiros com experiência suficiente, porém com salários mais compatíveis ao mercado local.
  - **Gestão de Projetos**: Avaliar a possibilidade de reduzir a carga horária do gestor de projetos ou redistribuir algumas responsabilidades para os engenheiros.

- **Parcerias e Patrocínios**:
  - Estabelecer parcerias com universidades ou empresas que possam fornecer componentes a custo reduzido em troca de publicidade ou participação no desenvolvimento do projeto.
  - Buscar patrocínios que cubram parte dos custos de hardware e mão de obra.

- **Ajustes no Escopo do Projeto**:
  - Revisar o escopo do projeto para identificar funcionalidades que podem ser simplificadas ou eliminadas sem comprometer a eficácia do PoC.

## Valor Mínimo Aceitável para Venda do Projeto

Para determinar o valor mínimo aceitável para a venda do projeto, consideramos a necessidade de cobrir os custos totais e ainda garantir uma margem mínima de lucro.

### Custos Reduzidos

Aplicando uma redução de 10% nos custos de hardware:

- **Novo Total de Hardware:** R$ 13.216,62 * 0,90 = R$ 11.894,96

Aplicando uma redução de 15% nos custos de mão de obra:

- **Novo Total de Mão de Obra:** R$ 65.500,00 * 0,85 = R$ 55.675,00

**Novo Custo Total:**

- **Custo Total Reduzido:** R$ 11.894,96 (hardware) + R$ 55.675,00 (mão de obra) = R$ 67.569,96

### Margem de Lucro

Para assegurar a viabilidade do projeto e um retorno mínimo aceitável, definimos uma margem de lucro de 10%.

| Descrição                       | Valor Original (R$) | Redução (%) | Novo Valor (R$) |
|---------------------------------|---------------------|-------------|-----------------|
| **Custos de Hardware**          | 13.216,62           | 10%         | 11.894,96       |
| **Custos de Mão de Obra**       | 65.500,00           | 15%         | 55.675,00       |
| **Custo Total Reduzido**        |                     |             | 67.569,96       |
| **Margem de Lucro (10%)**       |                     |             | 6.756,996       |
| **Valor Mínimo com Lucro**      |                     |             | 74.326,96       |


> Clique [aqui](https://1drv.ms/x/s!Arzvp0EDtndcgU2XKFVW1CqFbEmL?e=2Scuj8) para acessar a planilha Excel 
