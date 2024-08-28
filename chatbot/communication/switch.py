import subprocess

def main():
    # Avvia il secondo script usando Python 3
    proc = subprocess.Popen(
        ['python3', 'chat_process.py'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    try:
        # Leggi il messaggio di "alive" dallo script2
        alive_message = proc.stdout.readline().strip()
        if alive_message == "alive":
            print("Ricevuto messaggio: {}".format(alive_message))

        # Esegui il ciclo di invio/ricezione messaggi
        for i in range(5):
            messaggio = "Ciao, messaggio numero " + str(i+1) + "\n" 
            print("Invio: {}".format(messaggio.strip()))
            proc.stdin.write(messaggio)
            proc.stdin.flush()

            # Attendi la risposta
            risposta = proc.stdout.readline().strip()
            if risposta:
                print("Risposta: {}".format(risposta))
            else:
                print("Nessuna risposta ricevuta.")
                break

    except IOError as e:
        print("Errore durante la comunicazione: {}".format(e))

    finally:
        proc.stdin.close()
        proc.stdout.close()
        proc.stderr.close()
        proc.wait()

if __name__ == "__main__":
    main()
