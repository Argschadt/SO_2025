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


// ---------------------------------------------------------------------
// CONSTANTES E TIPOS {{{1
// ---------------------------------------------------------------------

// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  int regA, regX, regPC, regERRO; // cópia do estado da CPU
  // t2: tabela de processos, processo corrente, pendências, etc
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
} proc_t;

// tabela de processos
static proc_t proc_table[MAX_PROC];
// índice do processo corrente na tabela
static int proc_corrente_slot = -1;
// próximo pid a ser atribuído (init já tem pid 1)
static int next_pid = 2;

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
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
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
  // t2: realiza ações que não são diretamente ligadas com a interrupção que
  //   está sendo atendida:
  // - E/S pendente
  // - desbloqueio de processos
  // - contabilidades
  // - etc
}

static void so_escalona(so_t *self)
{
  // - se processo corrente  pronto/executando, continus
  // - senão, escolhe o primeiro processo na tabela que esteja PRONTO
  // - nenhum pronto, proc_corrente_slot = -1

  if (proc_corrente_slot != -1) {
    proc_state_t st = proc_table[proc_corrente_slot].state;
    if (st == PROC_PRONTO || st == PROC_EXECUTANDO) {
      proc_table[proc_corrente_slot].state = PROC_EXECUTANDO;
      return;
    }
  }

  // procura o primeiro processo pronto
  for (int i = 0; i < MAX_PROC; i++) {
    if (proc_table[i].state == PROC_PRONTO) {
      proc_corrente_slot = i;
      proc_table[i].state = PROC_EXECUTANDO;
      return;
    }
  }

  // nenhum processo pronto encontrado
  proc_corrente_slot = -1;
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
  proc_table[slot].state = PROC_PRONTO;
  proc_table[slot].PC = ender;
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].ERRO = 0;
  proc_table[slot].dev_in = D_TERM_A;
  proc_table[slot].dev_out = D_TERM_A;
  proc_table[slot].espera_pid = -1;
  proc_corrente_slot = slot;

  self->regA = proc_table[slot].A;
  self->regPC = proc_table[slot].PC;
  self->regERRO = proc_table[slot].ERRO;
  self->regX = proc_table[slot].X;
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
  // t2: deveria tratar a interrupção
  //   por exemplo, decrementa o quantum do processo corrente, quando se tem
  //   um escalonador com quantum
  console_printf("SO: interrupção do relógio (não tratada)");
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
  console_printf("SO: chamada de sistema %d", id_chamada);
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

  for (;;) {  // espera ocupada!
    int estado;
    if (es_le(self->es, dev_in + TERM_TECLADO_OK, &estado) != ERR_OK) {
      console_printf("SO: problema no acesso ao estado do teclado");
      self->erro_interno = true;
      return;
    }
    if (estado != 0) break;
    // como não está saindo do SO, a unidade de controle não está executando seu laço.
    // esta gambiarra faz pelo menos a console ser atualizada
    // t2: com a implementação de bloqueio de processo, esta gambiarra não
    //   deve mais existir.
    console_tictac(self->console);
  }
  int dado;
  if (es_le(self->es, dev_in + TERM_TECLADO, &dado) != ERR_OK) {
    console_printf("SO: problema no acesso ao teclado");
    self->erro_interno = true;
    return;
  }
  // escreve no reg A do processador
  // (na verdade, na posição onde o processador vai pegar o A quando retornar da int)
  // t2: se houvesse processo, deveria escrever no reg A do processo
  // t2: o acesso só deve ser feito nesse momento se for possível; se não, o processo
  //   é bloqueado, e o acesso só deve ser feito mais tarde (e o processo desbloqueado)
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

  for (;;) {
    int estado;
    if (es_le(self->es, dev_out + TERM_TELA_OK, &estado) != ERR_OK) {
      console_printf("SO: problema no acesso ao estado da tela");
      self->erro_interno = true;
      return;
    }
    if (estado != 0) break;
    // como não está saindo do SO, a unidade de controle não está executando seu laço.
    // esta gambiarra faz pelo menos a console ser atualizada
    // t2: não deve mais existir quando houver suporte a processos, porque o SO não poderá
    //   executar por muito tempo, permitindo a execução do laço da unidade de controle
    console_tictac(self->console);
  }
  int dado;
  // está lendo o valor de X e escrevendo o de A direto onde o processador colocou/vai pegar
  // t2: deveria usar os registradores do processo que está realizando a E/S
  // t2: caso o processo tenha sido bloqueado, esse acesso deve ser realizado em outra execução
  //   do SO, quando ele verificar que esse acesso já pode ser feito.
  dado = self->regX;
  if (es_escreve(self->es, dev_out + TERM_TELA, dado) != ERR_OK) {
    console_printf("SO: problema no acesso à tela");
    self->erro_interno = true;
    return;
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
  proc_table[slot].state = PROC_PRONTO;
  proc_table[slot].PC = ender_carga;
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].ERRO = 0;
  // herda dispositivos do processo chamador, se houver
  if (proc_corrente_slot != -1) {
    proc_table[slot].dev_in = proc_table[proc_corrente_slot].dev_in;
    proc_table[slot].dev_out = proc_table[proc_corrente_slot].dev_out;
  } else {
    proc_table[slot].dev_in = D_TERM_A;
    proc_table[slot].dev_out = D_TERM_A;
  }
  proc_table[slot].espera_pid = -1;

  // retorna o pid no registrador A do processo chamador
  self->regA = pid;
}

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
      return;
    }
  }

  // marca como morto e libera o slot
  proc_table[slot].state = PROC_MORTO;
  proc_table[slot].pid = 0;
  proc_table[slot].A = 0;
  proc_table[slot].X = 0;
  proc_table[slot].PC = 0;
  proc_table[slot].ERRO = 0;
  proc_table[slot].dev_in = -1;
  proc_table[slot].dev_out = -1;
  proc_table[slot].espera_pid = -1;

  if (slot == proc_corrente_slot) {
    proc_corrente_slot = -1;
  }

  self->regA = 0; // sucesso
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  // t2: deveria bloquear o processo se for o caso (e desbloquear na morte do esperado)
  // ainda sem suporte a processos, retorna erro -1
  console_printf("SO: SO_ESPERA_PROC não implementada");
  self->regA = -1;
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
