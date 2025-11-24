// so.c
// sistema operacional
// simulador de computador
// so25b

// ---------------------------------------------------------------------
// INCLUDES {{{1
// ---------------------------------------------------------------------

#include "so.h"
#include "dispositivos.h"
#include "err.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h> // para printf das métricas


// ---------------------------------------------------------------------
// CONSTANTES E TIPOS {{{1
// ---------------------------------------------------------------------

// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas
#define QUANTUM 5

// Métricas
typedef struct {
  int pid;
  int tempo_criacao;
  int tempo_termino;
  int preempcoes;
  int vezes_pronto;
  int vezes_executando;
  int vezes_bloqueado;
  int tempo_pronto;
  int tempo_executando;
  int tempo_bloqueado;
  int soma_tempo_resposta;
  int conta_tempo_resposta;
} metrica_proc_t;

#define MAX_METRICAS 100
static metrica_proc_t metricas_proc[MAX_METRICAS];
static int num_metricas = 0;

static int relogio_sistema = 0;
static int tempo_ocioso = 0;
static int total_preempcoes = 0;
static int total_processos_criados = 0;
static int cont_interrupcoes[8]; // IRQ_RESET=0 ... IRQ_RELOGIO=3 ...

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  int regA, regX, regPC, regERRO; // cópia do estado da CPU
  int contador_quantum;
};



// número máximo de procs
#define MAX_PROC 32

// estados do processo
typedef enum {
  PROC_VAZIO = 0,
  PROC_PRONTO,
  PROC_EXECUTANDO,
  PROC_BLOQUEADO,
  PROC_MORTO
} proc_state_t;

// Modos de escalonamento
typedef enum {
  ESCALONA_PRIORIDADE,
  ESCALONA_ROUND_ROBIN
} escalonamento_t;

// Variável para escolher o modo de escalonamento (padrão: PRIORIDADE)
static escalonamento_t modo_escalonamento = ESCALONA_ROUND_ROBIN;

// estrutura processo
typedef struct {
  int pid;
  proc_state_t state;
  int A;
  int X;
  int PC;
  int ERRO;
  int dev_in;
  int dev_out;
  int espera_pid;
  int waiting_dev;   // dispositivo que está aguardando (ou -1)
  int waiting_op;    // 0 = none, 1 = read, 2 = write
  double prioridade;
  
  // campos para métricas
  int id_metrica;
  int timestamp_estado; // tick do relógio quando entrou no estado atual
  int timestamp_pronto; // tick do relógio quando entrou em PRONTO
} proc_t;

// tabela de processos
static proc_t proc_table[MAX_PROC];
// índice do processo corrente na tabela
static int proc_corrente_slot = -1;
// próximo pid a ser atribuído (init já tem pid 1)
static int next_pid = 2;

// função auxiliar para mudança de estado e coleta de métricas
static void so_muda_estado(int slot, proc_state_t novo_estado) {
  if (slot < 0 || slot >= MAX_PROC) return;
  proc_t *p = &proc_table[slot];
  proc_state_t anterior = p->state;
  
  if (anterior == novo_estado) return;

  // Contabiliza tempo no estado anterior
  int delta = relogio_sistema - p->timestamp_estado;
  if (delta < 0) delta = 0; 
  
  int id = p->id_metrica;
  if (id >= 0 && id < MAX_METRICAS) {
    if (anterior == PROC_PRONTO) metricas_proc[id].tempo_pronto += delta;
    else if (anterior == PROC_EXECUTANDO) metricas_proc[id].tempo_executando += delta;
    else if (anterior == PROC_BLOQUEADO) metricas_proc[id].tempo_bloqueado += delta;
    
    // Contabiliza entrada no novo estado
    if (novo_estado == PROC_PRONTO) metricas_proc[id].vezes_pronto++;
    else if (novo_estado == PROC_EXECUTANDO) metricas_proc[id].vezes_executando++;
    else if (novo_estado == PROC_BLOQUEADO) metricas_proc[id].vezes_bloqueado++;
    
    // Tempo de resposta (PRONTO -> EXECUTANDO)
    if (anterior == PROC_PRONTO && novo_estado == PROC_EXECUTANDO) {
      int resp = relogio_sistema - p->timestamp_pronto;
      metricas_proc[id].soma_tempo_resposta += resp;
      metricas_proc[id].conta_tempo_resposta++;
    }
    
    // Marca entrada em PRONTO para cálculo futuro de resposta
    if (novo_estado == PROC_PRONTO) {
      p->timestamp_pronto = relogio_sistema;
    }
  }
  
  p->state = novo_estado;
  p->timestamp_estado = relogio_sistema;
}

// inicializa a tabela de processos (marca todos os slots como vazios)
static void init_proc_table(void)
{
  for (int i = 0; i < MAX_PROC; i++) {
    proc_table[i].pid = 0;
    proc_table[i].state = PROC_VAZIO;
    proc_table[i].A = 0;
    proc_table[i].X = 0;
    proc_table[i].PC = 0;
    proc_table[i].ERRO = 0;
    proc_table[i].dev_in = -1;
    proc_table[i].dev_out = -1;
    proc_table[i].espera_pid = -1;
    proc_table[i].waiting_dev = -1;
    proc_table[i].waiting_op = 0;
    proc_table[i].prioridade = 0.5; // prioridade inicial
    proc_table[i].id_metrica = -1;
    proc_table[i].timestamp_estado = 0;
    proc_table[i].timestamp_pronto = 0;
  }
  proc_corrente_slot = -1;
}



// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);


// ---------------------------------------------------------------------
// CRIAÇÃO {{{1
// ---------------------------------------------------------------------

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->contador_quantum = 0;

  // inicializa métricas globais
  relogio_sistema = 0;
  tempo_ocioso = 0;
  total_preempcoes = 0;
  total_processos_criados = 0;
  num_metricas = 0;
  for(int i=0; i<8; i++) cont_interrupcoes[i] = 0;

  /* inicializa tabela de processos */
  init_proc_table();

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  free(self);
}


// ---------------------------------------------------------------------
// TRATAMENTO DE INTERRUPÇÃO {{{1
// ---------------------------------------------------------------------

// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);
static void so_atualiza_prioridade(so_t *self, int slot); // recalcula prioridade

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
//   interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
//   a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
//   da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupção em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupção
static int so_trata_interrupcao(void *argC, int reg_A)
{
  so_t *self = argC;
  irq_t irq = reg_A;
  
  // contabiliza interrupção
  if (irq >= 0 && irq < 8) cont_interrupcoes[irq]++;
  
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupção
  so_trata_irq(self, irq);
  // faz o processamento independente da interrupção
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  so_escalona(self);
  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{
  // t2: salva os registradores que compõem o estado da cpu no descritor do
  //   processo corrente. os valores dos registradores foram colocados pela
  //   CPU na memória, nos endereços CPU_END_PC etc. O registrador X foi salvo
  //   pelo tratador de interrupção (ver trata_irq.asm) no endereço 59
  // se não houver processo corrente, não faz nada
  if (mem_le(self->mem, CPU_END_A, &self->regA) != ERR_OK
      || mem_le(self->mem, CPU_END_PC, &self->regPC) != ERR_OK
      || mem_le(self->mem, CPU_END_erro, &self->regERRO) != ERR_OK
      || mem_le(self->mem, 59, &self->regX)) {
    console_printf("SO: erro na leitura dos registradores");
    self->erro_interno = true;
    return;
  }

  /* se houver um processo corrente, salva os registradores no seu descritor */
  if (proc_corrente_slot != -1) {
    proc_table[proc_corrente_slot].A = self->regA;
    proc_table[proc_corrente_slot].PC = self->regPC;
    proc_table[proc_corrente_slot].ERRO = self->regERRO;
    proc_table[proc_corrente_slot].X = self->regX;
  }
}

static void so_trata_pendencias(so_t *self)
{
  // trata operações pendentes (E/S) de processos bloqueados
  // percorre a tabela procurando processos bloqueados e tenta completar
  // a operação se o dispositivo estiver pronto; em caso afirmativo,
  // realiza a E/S, limpa a pendência e marca o processo PRONTO.
  for (int i = 0; i < MAX_PROC; i++) {
    if (proc_table[i].state != PROC_BLOQUEADO) continue;
    int dev = proc_table[i].waiting_dev;
    int op = proc_table[i].waiting_op;
    if (dev == -1 || op == 0) continue;

    if (op == 1) { // leitura pendente
      int estado;
      if (es_le(self->es, dev + TERM_TECLADO_OK, &estado) != ERR_OK) {
        console_printf("SO: erro ao verificar estado do dispositivo %d", dev);
        self->erro_interno = true;
        continue;
      }
      if (estado != 0) {
        int dado;
        if (es_le(self->es, dev + TERM_TECLADO, &dado) != ERR_OK) {
          console_printf("SO: erro na leitura do dispositivo %d", dev);
          self->erro_interno = true;
          continue;
        }
        proc_table[i].A = dado;
        proc_table[i].waiting_dev = -1;
        proc_table[i].waiting_op = 0;
        so_muda_estado(i, PROC_PRONTO);
        console_printf("SO: processo pid=%d desbloqueado (leitura), dado=%d", proc_table[i].pid, dado);
      }

    } else if (op == 2) { // escrita pendente
      int estado;
      if (es_le(self->es, dev + TERM_TELA_OK, &estado) != ERR_OK) {
        console_printf("SO: erro ao verificar estado do dispositivo %d", dev);
        self->erro_interno = true;
        continue;
      }
      if (estado != 0) {
        if (es_escreve(self->es, dev + TERM_TELA, proc_table[i].X) != ERR_OK) {
          console_printf("SO: erro na escrita do dispositivo %d", dev);
          self->erro_interno = true;
          continue;
        }
        proc_table[i].A = 0;
        proc_table[i].waiting_dev = -1;
        proc_table[i].waiting_op = 0;
        so_muda_estado(i, PROC_PRONTO);
        console_printf("SO: processo pid=%d desbloqueado (escrita)", proc_table[i].pid);
      }
    }
  }
}

static void so_atualiza_prioridade(so_t *self, int slot)
{
  if (slot == -1) return;
  double t_exec = QUANTUM - self->contador_quantum;
  // garante que não seja negativo ou maior que quantum por algum erro
  if (t_exec < 0) t_exec = 0;
  if (t_exec > QUANTUM) t_exec = QUANTUM;
  
  double uso = t_exec / (double)QUANTUM;
  proc_table[slot].prioridade = (proc_table[slot].prioridade + uso) / 2.0;
}

static void so_escalona(so_t *self)
{
  // se o processo corrente ainda está executando, continua com ele
  if (proc_corrente_slot != -1) {
    proc_state_t st = proc_table[proc_corrente_slot].state;
    if (st == PROC_EXECUTANDO) {
      return;
    }
  }

  int escolhido = -1;

  if (modo_escalonamento == ESCALONA_PRIORIDADE) {
    // escalonador com prioridade
    // escolhe o processo pronto com a menor prioridade (menor valor = maior prioridade)
    double melhor_prio = 1000000.0; // valor alto inicial

    for (int i = 0; i < MAX_PROC; i++) {
      if (proc_table[i].state == PROC_PRONTO) {
        if (escolhido == -1 || proc_table[i].prioridade < melhor_prio) {
          escolhido = i;
          melhor_prio = proc_table[i].prioridade;
        }
      }
    }
  } else {
    // escalonador round-robin
    // escolhe o próximo processo pronto na tabela, circularmente
    int inicio = (proc_corrente_slot + 1) % MAX_PROC;
    // Se proc_corrente_slot for -1, inicio será 0.
    if (inicio < 0) inicio = 0;

    for (int i = 0; i < MAX_PROC; i++) {
      int idx = (inicio + i) % MAX_PROC;
      if (proc_table[idx].state == PROC_PRONTO) {
        escolhido = idx;
        break;
      }
    }
  }
  
  if (escolhido != -1) {
    proc_corrente_slot = escolhido;
    so_muda_estado(escolhido, PROC_EXECUTANDO);
    self->contador_quantum = QUANTUM; // reinicia o quantum
  } else {
    // nenhum processo pronto
    proc_corrente_slot = -1;
  }
}

static int so_despacha(so_t *self)
{
  // t2: se houver processo corrente, coloca o estado desse processo onde ele
  //   será recuperado pela CPU (em CPU_END_PC etc e 59) e retorna 0,
  //   senão retorna 1
  // o valor retornado será o valor de retorno de CHAMAC, e será colocado no 
  //   registrador A para o tratador de interrupção (ver trata_irq.asm).
  if (proc_corrente_slot == -1) {
    return 1;
  }

  /* coloca no buffer do SO os registradores do processo corrente antes de
     escrever na memória onde o tratador em asm espera encontrá-los */
  self->regA = proc_table[proc_corrente_slot].A;
  self->regPC = proc_table[proc_corrente_slot].PC;
  self->regERRO = proc_table[proc_corrente_slot].ERRO;
  self->regX = proc_table[proc_corrente_slot].X;

  if (mem_escreve(self->mem, CPU_END_A, self->regA) != ERR_OK
      || mem_escreve(self->mem, CPU_END_PC, self->regPC) != ERR_OK
      || mem_escreve(self->mem, CPU_END_erro, self->regERRO) != ERR_OK
      || mem_escreve(self->mem, 59, self->regX)) {
    console_printf("SO: erro na escrita dos registradores");
    self->erro_interno = true;
  }
  if (self->erro_interno) return 1;
  else return 0;
}


// ---------------------------------------------------------------------
// TRATAMENTO DE UMA IRQ {{{1
// ---------------------------------------------------------------------

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupção que está acontecendo, e atende de acordo
  switch (irq) {
    case IRQ_RESET:
      so_trata_reset(self);
      break;
    case IRQ_SISTEMA:
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      so_trata_irq_relogio(self);
      break;
    default:
      so_trata_irq_desconhecida(self, irq);
  }
}

// chamada uma única vez, quando a CPU inicializa
static void so_trata_reset(so_t *self)
{
  // coloca o tratador de interrupção na memória
  // quando a CPU aceita uma interrupção, passa para modo supervisor,
  //   salva seu estado à partir do endereço CPU_END_PC, e desvia para o
  //   endereço CPU_END_TRATADOR
  // colocamos no endereço CPU_END_TRATADOR o programa de tratamento
  //   de interrupção (escrito em asm). esse programa deve conter a
  //   instrução CHAMAC, que vai chamar so_trata_interrupcao (como
  //   foi definido na inicialização do SO)
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != CPU_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  // t2: deveria criar um processo para o init, e inicializar o estado do
  //   processador para esse processo com os registradores zerados, exceto
  //   o PC e o modo.
  // como não tem suporte a processos, está carregando os valores dos
  //   registradores diretamente no estado da CPU mantido pelo SO; daí vai
  //   copiar para o início da memória pelo despachante, de onde a CPU vai
  //   carregar para os seus registradores quando executar a instrução RETI
  //   em bios.asm (que é onde está a instrução CHAMAC que causou a execução
  //   deste código

  // coloca o programa init na memória
  ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  // cria o descritor do primeiro processo (init) e inicializa a tabela
  int slot = 0;
  proc_table[slot].pid = 1;
  // inicializa métrica do init
  if (num_metricas < MAX_METRICAS) {
    int id = num_metricas++;
    metricas_proc[id].pid = 1;
    metricas_proc[id].tempo_criacao = relogio_sistema;
    metricas_proc[id].preempcoes = 0;
    metricas_proc[id].vezes_pronto = 1; // já nasce pronto
    metricas_proc[id].vezes_executando = 0;
    metricas_proc[id].vezes_bloqueado = 0;
    metricas_proc[id].tempo_pronto = 0;
    metricas_proc[id].tempo_executando = 0;
    metricas_proc[id].tempo_bloqueado = 0;
    metricas_proc[id].soma_tempo_resposta = 0;
    metricas_proc[id].conta_tempo_resposta = 0;
    proc_table[slot].id_metrica = id;
    total_processos_criados++;
  }
  
  proc_table[slot].state = PROC_PRONTO; // usa direto pois so_muda_estado usa timestamp anterior
  proc_table[slot].timestamp_estado = relogio_sistema;
  proc_table[slot].timestamp_pronto = relogio_sistema;
  
  proc_table[slot].PC = ender;
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].ERRO = 0;
  proc_table[slot].dev_in = D_TERM_A;
  proc_table[slot].dev_out = D_TERM_A;
  proc_table[slot].espera_pid = -1;
  
  // insere init na fila de prontos e deixa o escalonador escolher
  proc_corrente_slot = -1;
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  // Ocorreu um erro interno na CPU
  // O erro está codificado em CPU_END_erro
  // Em geral, causa a morte do processo que causou o erro
  // Ainda não temos processos, causa a parada da CPU
  // t2: com suporte a processos, deveria pegar o valor do registrador erro
  //   no descritor do processo corrente, e reagir de acordo com esse erro
  //   (em geral, matando o processo)
  err_t err = self->regERRO;
  console_printf("SO: IRQ não tratada -- erro na CPU: %s", err_nome(err));
  self->erro_interno = true;
}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // rearma o interruptor do relógio e reinicializa o timer para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }

  // contabiliza tempo do sistema
  relogio_sistema++;

  // t3: contabiliza tempo ocioso se não houver processo executando
  if (proc_corrente_slot == -1) {
    tempo_ocioso++;
  }
  
  // quantum
  if (proc_corrente_slot != -1) {
    self->contador_quantum--;
    if (self->contador_quantum <= 0) {
      // preempção
      so_atualiza_prioridade(self, proc_corrente_slot); // recalcula prioridade
      
      // contabiliza preempção
      total_preempcoes++;
      int id = proc_table[proc_corrente_slot].id_metrica;
      if (id >= 0 && id < MAX_METRICAS) {
        metricas_proc[id].preempcoes++;
      }

      // muda estado usando a função auxiliar para contabilizar métricas
      so_muda_estado(proc_corrente_slot, PROC_PRONTO);
      
      // O escalonador será chamado em seguida no so_trata_interrupcao
      proc_corrente_slot = -1; // força escolha de novo processo
    }
  }
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}


// ---------------------------------------------------------------------
// CHAMADAS DE SISTEMA {{{1
// ---------------------------------------------------------------------

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  // t2: com processos, o reg A deve estar no descritor do processo corrente
  int id_chamada = self->regA;
  switch (id_chamada) {
    case SO_LE:
      so_chamada_le(self);
      break;
    case SO_ESCR:
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);
      // t2: deveria matar o processo
      self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  // implementação com espera ocupada
  //   t2: deveria realizar a leitura somente se a entrada estiver disponível,
  //     senão, deveria bloquear o processo.
  //   no caso de bloqueio do processo, a leitura (e desbloqueio) deverá
  //     ser feita mais tarde, em tratamentos pendentes em outra interrupção,
  //     ou diretamente em uma interrupção específica do dispositivo, se for
  //     o caso
  // implementação lendo direto do terminal A
  //   t2: deveria usar dispositivo de entrada corrente do processo
  /* determina o dispositivo de entrada do processo corrente */
  int dev_in = D_TERM_A;
  if (proc_corrente_slot != -1 && proc_table[proc_corrente_slot].dev_in != -1) {
    dev_in = proc_table[proc_corrente_slot].dev_in;
  }
  /* verifica uma vez se o dispositivo está pronto; se não, bloqueia o processo */
  int estado;
  if (es_le(self->es, dev_in + TERM_TECLADO_OK, &estado) != ERR_OK) {
    console_printf("SO: problema no acesso ao estado do teclado");
    self->erro_interno = true;
    return;
  }

  if (estado == 0) {
    /* bloqueia o processo atual aguardando leitura do dispositivo */
    if (proc_corrente_slot != -1) {
      so_atualiza_prioridade(self, proc_corrente_slot); // recalcula prioridade antes de bloquear
      so_muda_estado(proc_corrente_slot, PROC_BLOQUEADO);
      proc_table[proc_corrente_slot].waiting_dev = dev_in;
      proc_table[proc_corrente_slot].waiting_op = 1;
    }
    return;
  }

  /* dispositivo pronto: faz a leitura imediatamente */
  int dado;
  if (es_le(self->es, dev_in + TERM_TECLADO, &dado) != ERR_OK) {
    console_printf("SO: problema no acesso ao teclado");
    self->erro_interno = true;
    return;
  }
  /* atualiza o descritor do processo corrente com o valor lido */
  if (proc_corrente_slot != -1) {
    proc_table[proc_corrente_slot].A = dado;
    proc_table[proc_corrente_slot].waiting_dev = -1;
    proc_table[proc_corrente_slot].waiting_op = 0;
  }
  self->regA = dado;
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
  // implementação com espera ocupada
  //   t2: deveria bloquear o processo se dispositivo ocupado
  // implementação escrevendo direto do terminal A
  //   t2: deveria usar o dispositivo de saída corrente do processo
  /* determina o dispositivo de saída do processo corrente */
  int dev_out = D_TERM_A;
  if (proc_corrente_slot != -1 && proc_table[proc_corrente_slot].dev_out != -1) {
    dev_out = proc_table[proc_corrente_slot].dev_out;
  }

  /* verifica uma vez se a tela está pronta; se não, bloqueia o processo */
  int estado_out;
  if (es_le(self->es, dev_out + TERM_TELA_OK, &estado_out) != ERR_OK) {
    console_printf("SO: problema no acesso ao estado da tela");
    self->erro_interno = true;
    return;
  }

  if (estado_out == 0) {
    /* bloqueia o processo atual aguardando escrita no dispositivo */
    if (proc_corrente_slot != -1) {
      so_atualiza_prioridade(self, proc_corrente_slot); // recalcula prioridade antes de bloquear
      so_muda_estado(proc_corrente_slot, PROC_BLOQUEADO);
      proc_table[proc_corrente_slot].waiting_dev = dev_out;
      proc_table[proc_corrente_slot].waiting_op = 2; /* write */
      /* garante que o valor a ser escrito está no descritor (X) */
      proc_table[proc_corrente_slot].X = self->regX;
    }
    return;
  }

  /* dispositivo pronto: realiza a escrita */
  int dado = self->regX;
  if (es_escreve(self->es, dev_out + TERM_TELA, dado) != ERR_OK) {
    console_printf("SO: problema no acesso à tela");
    self->erro_interno = true;
    return;
  }
  if (proc_corrente_slot != -1) {
    proc_table[proc_corrente_slot].A = 0;
    proc_table[proc_corrente_slot].waiting_dev = -1;
    proc_table[proc_corrente_slot].waiting_op = 0;
  }
  self->regA = 0;
}

// implementação da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // cria um novo processo a partir do nome do executavel apontado por X
  int ender_proc = self->regX; // endereço na memória do chamador com o nome
  char nome[100];

  if (!copia_str_da_mem(100, nome, self->mem, ender_proc)) {
    console_printf("SO: nome do executavel invalido na criacao de processo");
    self->regA = -1;
    return;
  }

  // encontra um slot livre
  int slot = -1;
  for (int i = 0; i < MAX_PROC; i++) {
    if (proc_table[i].state == PROC_VAZIO) {
      slot = i;
      break;
    }
  }
  if (slot == -1) {
    console_printf("SO: nenhum slot disponivel para novo processo");
    self->regA = -1;
    return;
  }

  // carrega o programa na memoria
  int ender_carga = so_carrega_programa(self, nome);
  if (ender_carga < 0) {
    console_printf("SO: falha na carga do programa '%s' ao criar processo", nome);
    self->regA = -1;
    return;
  }

  // atribui pid e inicializa o descritor
  int pid = next_pid++;
  proc_table[slot].pid = pid;
  
  // inicializa métrica
  if (num_metricas < MAX_METRICAS) {
    int id = num_metricas++;
    metricas_proc[id].pid = pid;
    metricas_proc[id].tempo_criacao = relogio_sistema;
    metricas_proc[id].preempcoes = 0;
    metricas_proc[id].vezes_pronto = 1; // já nasce pronto
    metricas_proc[id].vezes_executando = 0;
    metricas_proc[id].vezes_bloqueado = 0;
    metricas_proc[id].tempo_pronto = 0;
    metricas_proc[id].tempo_executando = 0;
    metricas_proc[id].tempo_bloqueado = 0;
    metricas_proc[id].soma_tempo_resposta = 0;
    metricas_proc[id].conta_tempo_resposta = 0;
    proc_table[slot].id_metrica = id;
    total_processos_criados++;
  } else {
    proc_table[slot].id_metrica = -1;
  }

  proc_table[slot].state = PROC_PRONTO; // usa direto
  proc_table[slot].timestamp_estado = relogio_sistema;
  proc_table[slot].timestamp_pronto = relogio_sistema;
  
  proc_table[slot].PC = ender_carga;
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].ERRO = 0;
  // atribui terminal exclusivo baseado no pid (não herda)
  int term_index = (pid - 1) % 4; // pid=1 -> 0 (A), pid=2 -> 1 (B), pid=3 -> 2 (C), pid=4 -> 3 (D), pid=5 -> 0 (A), etc.
  proc_table[slot].dev_in = D_TERM_A + term_index * 4;
  proc_table[slot].dev_out = D_TERM_A + term_index * 4;
  proc_table[slot].espera_pid = -1;
  proc_table[slot].prioridade = 0.5; // t3: prioridade inicial

  // retorna o pid no registrador A do processo chamador
  self->regA = pid;
  if (proc_corrente_slot != -1) {
    proc_table[proc_corrente_slot].A = pid;
  }
}

static void so_imprime_metricas(so_t *self);

// implementação da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  // mata um processo identificado pelo pid em X (ou 0 para o processo atual)
  int pid = self->regX;
  int slot = -1;

  if (pid == 0) {
    if (proc_corrente_slot == -1) {
      self->regA = -1; // não há processo corrente
      return;
    }
    slot = proc_corrente_slot;
  } else {
    for (int i = 0; i < MAX_PROC; i++) {
      if (proc_table[i].pid == pid && proc_table[i].state != PROC_VAZIO) {
        slot = i;
        break;
      }
    }
    if (slot == -1) {
      // pid inexistente
      self->regA = -1;
      if (proc_corrente_slot != -1) proc_table[proc_corrente_slot].A = -1;
      return;
    }
  }

  // finaliza métrica
  int id = proc_table[slot].id_metrica;
  if (id >= 0 && id < MAX_METRICAS) {
    metricas_proc[id].tempo_termino = relogio_sistema;
    // Atualiza tempo final no estado EXECUTANDO (se for ele mesmo morrendo)
    // Se for outro processo matando, ele pode estar em outro estado.
    // so_muda_estado vai contabilizar o tempo no estado atual antes de mudar para MORTO
  }
  
  // marca como morto
  so_muda_estado(slot, PROC_MORTO);
  
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].PC = 0;
  proc_table[slot].ERRO = 0;
  proc_table[slot].dev_in = -1;
  proc_table[slot].dev_out = -1;

  // acorda quaisquer processos que estavam esperando por este pid
  int pid_morto = proc_table[slot].pid;
  for (int j = 0; j < MAX_PROC; j++) {
    if (proc_table[j].state == PROC_BLOQUEADO && proc_table[j].espera_pid == pid_morto) {
      proc_table[j].espera_pid = -1;
      so_muda_estado(j, PROC_PRONTO);
      proc_table[j].A = 0;
      console_printf("SO: acordando pid=%d que esperava pid=%d", proc_table[j].pid, pid_morto);
    }
  }

  if (slot == proc_corrente_slot) {
    proc_corrente_slot = -1;
  } else {
    // se matou outro processo, retorna sucesso para o chamador
    self->regA = 0;
    if (proc_corrente_slot != -1) proc_table[proc_corrente_slot].A = 0;
  }
  
  // se init morreu, imprime métricas
  if (pid_morto == 1) {
	so_imprime_metricas(self);
  }

  self->regA = 0; // sucesso (caso geral, mas se morreu não importa)
}

static void so_imprime_metricas(so_t *self) {
  // lê o tempo real de execução do simulador em ms a partir do dispositivo de relógio
  int tempo_real_ms = 0;
  if (es_le(self->es, D_RELOGIO_REAL, &tempo_real_ms) != ERR_OK) {
    tempo_real_ms = -1; // indica erro de leitura
  }

  printf("\r\n=== METRICAS DO SISTEMA ===\r\n");
  printf("Processos criados: %d\r\n", total_processos_criados);
  printf("Tempo total de execucao (ticks): %d\r\n", relogio_sistema);
  if (tempo_real_ms >= 0) {
    printf("Tempo total de execucao real: %d ms (%.3f s)\r\n", tempo_real_ms, tempo_real_ms / 1000.0);
  }
  printf("Tempo ocioso: %d ticks\r\n", tempo_ocioso);
  printf("Total de preempcoes: %d\r\n", total_preempcoes);
  printf("Interrupcoes: RESET=%d, SIS=%d, ERR=%d, REL=%d\r\n", 
         cont_interrupcoes[IRQ_RESET], cont_interrupcoes[IRQ_SISTEMA], 
         cont_interrupcoes[IRQ_ERR_CPU], cont_interrupcoes[IRQ_RELOGIO]);
	
  printf("\r\n--- Metricas por Processo ---\r\n");
  printf("PID | Retorno | Preemp | Prnt(n/t) | Exec(n/t) | Bloq(n/t) | Resp(med)\r\n");
  for (int i = 0; i < num_metricas; i++) {
    metrica_proc_t *m = &metricas_proc[i];
    int retorno = m->tempo_termino - m->tempo_criacao;
    double resp_media = m->conta_tempo_resposta > 0 ? 
                        (double)m->soma_tempo_resposta / m->conta_tempo_resposta : 0.0;
	  
    printf("%3d | %7d | %6d | %2d/%5d | %2d/%5d | %2d/%5d | %8.2f\r\n",
           m->pid, retorno, m->preempcoes,
           m->vezes_pronto, m->tempo_pronto,
           m->vezes_executando, m->tempo_executando,
           m->vezes_bloqueado, m->tempo_bloqueado,
           resp_media);
  }
  printf("===========================\r\n");
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  int pid = self->regX; // pid to wait for
  if (proc_corrente_slot == -1) {
    self->regA = -1;
    return;
  }
  int caller = proc_corrente_slot;
  // não pode esperar por si mesmo
  if (pid == proc_table[caller].pid || pid == 0) {
    self->regA = -1;
    proc_table[caller].A = -1;
    return;
  }
  // procura o processo com pid
  int target_slot = -1;
  for (int i = 0; i < MAX_PROC; i++) {
    if (proc_table[i].pid == pid && proc_table[i].state != PROC_VAZIO) {
      target_slot = i;
      break;
    }
  }
  if (target_slot == -1) {
    // processo inexistente
    self->regA = -1;
    proc_table[caller].A = -1;
    return;
  }
  // se o processo já está morto, retorna sucesso imediatamente
  if (proc_table[target_slot].state == PROC_MORTO) {
    self->regA = 0;
    proc_table[caller].A = 0;
    return;
  }
  // bloqueia o chamador até que o processo com pid termine
  so_atualiza_prioridade(self, caller); // recalcula prioridade antes de bloquear
  so_muda_estado(caller, PROC_BLOQUEADO);
  proc_table[caller].espera_pid = pid;
  // o retorno será fornecido quando o processo alvo morrer (so_chamada_mata_proc)
  return;
}


// ---------------------------------------------------------------------
// CARGA DE PROGRAMA {{{1
// ---------------------------------------------------------------------

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}


// ---------------------------------------------------------------------
// ACESSO À MEMÓRIA DOS PROCESSOS {{{1
// ---------------------------------------------------------------------

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// t2: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

// vim: foldmethod=marker
